
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  nh.param("fsm/finish_recheck_interval", finish_recheck_interval_, 2.0);
  nh.param("fsm/finish_recheck_max", finish_recheck_max_, 15);
  finish_recheck_count_ = 0;
  finish_last_recheck_ = ros::Time::now();

  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
  // The mission layer (EDM) decides the run is over on a coverage plateau, which can happen
  // while our own frontier logic still has work queued. Without this handshake the EDM flips
  // to RETURN and starts publishing /planning/pos_cmd while we are still publishing it too --
  // measured on runs 20260909_082915 and 20260909_101738. On the latter the plateau fired at
  // t=505.4 s and we did not finish until t=594.5 s: 89 s of both of us driving the topic,
  // coverage frozen at 99.24 %, then FAST-LIO diverged at t=600 and PX4 failsafed the vehicle
  // down 9.1 m from the pad. One publisher at a time.
  stop_sub_ = nh.subscribe("/mission/stop_exploration", 1,
                           &FastExplorationFSM::stopExplorationCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  // Latched: the EDM may well subscribe after we have already finished, and a one-shot
  // non-latched Bool published into an empty subscriber list is simply lost - which would
  // strand the vehicle hovering inside the arena with the mission believing it is still
  // exploring. queue 1 + latch means a late subscriber still gets it.
  completed_pub_ = nh.advertise<std_msgs::Bool>("/exploration_completed", 1, true);
  next_view_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/exploration/next_view", 10);
  exploration_completed_sent_ = false;
  stop_requested_ = false;
}

// Clamp a MEASURED velocity to something the kinodynamic search can actually start from.
//
// KinodynamicAstar rejects any start state whose speed exceeds max_vel + vel_margin
// (0.6 + 0.25 = 0.85 here) and returns NO_PATH immediately. FAST-LIO's ESEKF velocity is an
// estimate, not the commanded value, and it overshoots: measured 2026-09-06 over one flight,
// the failing replans had a median start speed of 0.933 m/s and 60 of 90 were above 0.85,
// while ZERO of the 44 successful replans were -- a perfect separation. Plan success sat at
// 30% purely because of it.
//
// The vehicle is commanded never to exceed max_vel, so an estimate above it is noise, not
// motion. Clamp the magnitude and keep the direction, which is the part the planner and
// ViewNode::computeCost's direction term actually need. Clamping to max_vel (not to
// max_vel + vel_margin) deliberately leaves the margin as headroom for genuine overshoot.
Eigen::Vector3d FastExplorationFSM::clampStartVel(const Eigen::Vector3d& v) const {
  const double vmax = planner_manager_->pp_.max_vel_;
  const double n = v.norm();
  if (!std::isfinite(n)) return Eigen::Vector3d::Zero();
  if (n <= vmax || n < 1e-6) return v;
  return v * (vmax / n);
}

// Vertical half of the guard above. The magnitude clamp cannot see this case: a start velocity
// that is small overall but almost entirely VERTICAL passes it untouched and still kills the
// kinodynamic search, because the planning box is only 0.50 m tall (box_z 1.09..1.59).
//
// KinodynamicAstar prunes any motion primitive whose sampled trajectory leaves the box
// (kinodynamic_astar.cpp:168-172). Climbing at vz needs vz^2/(2*a) of height to arrest, so the
// search can only survive if that fits in the remaining headroom:
//
//     |vz| <= sqrt(2 * max_acc * headroom)
//
// Measured 2026-09-09 against the recorded failing case from run 20260909_062048 (start
// 9.647 -1.602 1.385, vel 0.006 0.027 0.557, goal 9.584 0.452 1.239): the vehicle needed
// 0.517 m to arrest but had 0.205 m of headroom, so 888 of 895 generated primitives were
// pruned by the box test, the open set emptied, and the search returned NO_PATH on EVERY
// replan for 14 minutes. Offline sweep of the real expansion loop confirms the relation
// exactly -- failure onset at vz 0.35 (headroom 0.205, a 0.3), 0.50 (a 0.6), 0.56
// (headroom 0.515) -- and confirms this clamp restores a path at every (altitude, vz) tested.
//
// A finer primitive-duration ladder does NOT help and was rejected on that evidence: the limit
// is kinematic, not a search-resolution artifact.
//
// The velocity being clamped is largely not real motion in the first place -- it is FAST-LIO
// accelerometer-bias drift leaking into the ESEKF velocity channel (ba reached 0.46 m/s^2 on
// that run) while ground truth showed no vertical translation. Fixing that at source is the
// proper repair; this keeps the planner alive until it lands.
Eigen::Vector3d FastExplorationFSM::clampStartVelZ(const Eigen::Vector3d& p,
                                                   const Eigen::Vector3d& v) const {
  Eigen::Vector3d out = v;
  if (!std::isfinite(v.z()) || !std::isfinite(p.z())) {
    out.z() = 0.0;
    return out;
  }
  Eigen::Vector3d bmin, bmax;
  planner_manager_->edt_environment_->sdf_map_->getBox(bmin, bmax);

  // Headroom in the direction of travel only: climbing is limited by the ceiling, descending
  // by the floor.
  const double head = (v.z() > 0.0) ? (bmax.z() - p.z()) : (p.z() - bmin.z());
  if (head <= 0.0) {
    out.z() = 0.0;   // already outside the box vertically; any vertical rate makes it worse
    return out;
  }
  // 0.8 keeps a margin against the box edge rather than aiming to stop exactly on it.
  const double vz_max = 0.8 * std::sqrt(2.0 * planner_manager_->pp_.max_acc_ * head);
  if (std::fabs(out.z()) > vz_max) {
    ROS_WARN_THROTTLE(2.0,
                      "[FSM] start vz %.3f m/s exceeds what %.2f m of box headroom allows "
                      "(%.3f m/s at a=%.2f); clamping so the kinodynamic search can plan.",
                      out.z(), head, vz_max, planner_manager_->pp_.max_acc_);
    out.z() = std::copysign(vz_max, out.z());
  }
  return out;
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      // Do not accept the first NO_FRONTIER as the end of the mission -- see the comment on
      // finish_recheck_* in the header. Re-attempt planning a bounded number of times; the
      // map keeps growing from the sensor stream while we wait, so frontiers that did not
      // exist at the moment of the trigger will appear.
      if (finish_recheck_count_ < finish_recheck_max_ &&
          (ros::Time::now() - finish_last_recheck_).toSec() > finish_recheck_interval_) {
        finish_last_recheck_ = ros::Time::now();
        ++finish_recheck_count_;
        ROS_WARN("[FSM] FINISH re-check %d/%d: retrying exploration planning.",
                 finish_recheck_count_, finish_recheck_max_);
        transitState(PLAN_TRAJ, "FSM");
        break;
      }
      // Re-checks exhausted: this is a real finish, not a momentary frontier gap. Tell the
      // mission layer exactly once, then keep holding.
      if (!exploration_completed_sent_) {
        exploration_completed_sent_ = true;
        std_msgs::Bool done;
        done.data = true;
        completed_pub_.publish(done);
        ROS_WARN("[FSM] exploration COMPLETE after %d re-checks: publishing "
                 "/exploration_completed for the return leg.", finish_recheck_count_);
      }
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = clampStartVel(fd_->odom_vel_);
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        // BOUND t_r TO THE TRAJECTORY. NonUniformBspline::evaluateDeBoorT does not clamp its
        // argument, so once t_r runs past duration_ -- which happens whenever a replan is late,
        // and always once the vehicle has flown the whole trajectory while the planner kept
        // failing -- the B-spline is evaluated outside its knot span and EXTRAPOLATES.
        //
        // Measured 2026-09-06 with this unbounded: the start velocity handed to the
        // kinodynamic search had a median of 0.881 m/s, p90 of 3.12 and a maximum of 4.95 m/s
        // on a vehicle whose max_vel is 0.6 -- physically impossible values, produced entirely
        // by extrapolation. KinodynamicAstar rejects any start state above
        // max_vel + vel_margin (0.85), so 87 of 173 replans were refused outright and plan
        // success sat at 32%. The vehicle showed it as stopping and restarting, because a
        // refused replan leaves it holding the previous trajectory's endpoint.
        //
        // Clamping the magnitude afterwards is not a substitute: an extrapolated velocity has
        // a meaningless DIRECTION too, and the direction feeds both the search and
        // ViewNode::computeCost's w_dir term. Bound the evaluation instead, so the state is
        // the real end-of-trajectory state.
        t_r = std::min(t_r, info->duration_);

        Eigen::Vector3d predicted_pt = info->position_traj_.evaluateDeBoorT(t_r);

        // Sanity-check the predicted start state against real odometry (fd_->odom_pos_ is kept
        // up to date by odometryCallback regardless of static_state_). If the vehicle isn't
        // actually tracking the planned trajectory -- stalled, physically obstructed, briefly
        // lost aiding, anything -- replanning from the trajectory's own prediction compounds the
        // divergence indefinitely, since it's never corrected against reality except on a full
        // static-state reset (which only happens after NO_FRONTIER or a hard planning FAIL).
        // Left unchecked, the planner can keep computing short, "almost there" trajectories
        // relative to its own drifting internal belief while the real vehicle never moves.
        const double kMaxTrackingError = 1.0;  // meters
        if ((predicted_pt - fd_->odom_pos_).norm() > kMaxTrackingError) {
          ROS_WARN(
              "[FSM] Trajectory tracking diverged from real odometry by %.2fm -- replanning "
              "from real odometry instead of predicted trajectory state",
              (predicted_pt - fd_->odom_pos_).norm());
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = clampStartVel(fd_->odom_vel_);
          fd_->start_acc_.setZero();
          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
        } else {
          fd_->start_pt_ = predicted_pt;
          fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
          fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
          fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
        }
      }

      // ONE clamp covering every branch above. The B-spline optimiser enforces max_vel as a
      // SOFT cost, so info->velocity_traj_ can exceed it too -- and since 86% of replans take
      // the trajectory branch, clamping only the odometry branch would have left most of the
      // failures in place. Measured before this clamp: 90 of 119 replans failed, 60 of those
      // with a start speed over the search's 0.85 m/s ceiling, against 0 of 44 successes.
      fd_->start_vel_ = clampStartVel(fd_->start_vel_);
      // Start HEIGHT guard. The map box is only 0.50 m tall, and a start point outside it makes
      // KinodynamicAstar fail on its very first node ("open set empty, use node num: 1"). Run
      // 20261001_161018: a height-estimate error (rangefinder reading survivor tops as floor)
      // put odometry z at 1.78-1.87 against box_max_z 1.59, and 18771 of the run's replans
      // failed that way while coverage stalled at 52% for 290 s. Planning at a clamped height
      // is always better than not planning at all: traj_server flies its own z_cruise anyway.
      {
        Eigen::Vector3d bmin, bmax;
        planner_manager_->edt_environment_->sdf_map_->getBox(bmin, bmax);
        const double margin = 0.05;
        const double z_lo = bmin.z() + margin, z_hi = bmax.z() - margin;
        if (std::isfinite(fd_->start_pt_.z()) && z_lo < z_hi &&
            (fd_->start_pt_.z() < z_lo || fd_->start_pt_.z() > z_hi)) {
          ROS_WARN_THROTTLE(2.0,
                            "[FSM] start z %.2f outside map box [%.2f, %.2f]; clamping so the "
                            "kinodynamic search can expand.",
                            fd_->start_pt_.z(), bmin.z(), bmax.z());
          fd_->start_pt_.z() = std::min(std::max(fd_->start_pt_.z(), z_lo), z_hi);
        }
      }
      // Vertical guard, same single-point placement and for the same reason: the box is only
      // 0.50 m tall, so a vertical rate the magnitude clamp happily passes can still prune
      // every motion primitive. See clampStartVelZ.
      fd_->start_vel_ = clampStartVelZ(fd_->start_pt_, fd_->start_vel_);

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        // A successful plan means exploration is live again; allow the full re-check budget
        // if we ever land back in FINISH later in the mission.
        finish_recheck_count_ = 0;
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        // This is the authoritative "exploration actually advanced" signal, so it is what the
        // global stall clock keys off.
        expl_manager_->ed_->last_progress_time_ = ros::Time::now();
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    // Timestamped record of the committed viewpoint, for post-run analysis. last_next_pos_ is
    // the viewpoint planExploreMotion just committed to (it writes it immediately before the
    // "Next view:" print), so this is the same target, with a timestamp attached.
    geometry_msgs::PoseStamped nv;
    nv.header.stamp = ros::Time::now();
    nv.header.frame_id = "world";
    nv.pose.position.x = expl_manager_->ed_->last_next_pos_(0);
    nv.pose.position.y = expl_manager_->ed_->last_next_pos_(1);
    nv.pose.position.z = expl_manager_->ed_->last_next_pos_(2);
    nv.pose.orientation.z = sin(0.5 * expl_manager_->ed_->last_next_yaw_);
    nv.pose.orientation.w = cos(0.5 * expl_manager_->ed_->last_next_yaw_);
    next_view_pub_.publish(nv);
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::stopExplorationCallback(const std_msgs::BoolConstPtr& msg) {
  if (!msg->data || stop_requested_) return;
  stop_requested_ = true;
  // Go straight to FINISH and skip the re-check budget. The re-checks exist to survive a
  // MOMENTARY frontier gap; this is not that -- the mission layer has decided on evidence we
  // do not have (a coverage plateau) that there is nothing left worth flying to, so retrying
  // exploration planning would only keep us publishing trajectories the mission no longer
  // wants. FINISH then publishes /exploration_completed exactly once, which is the signal the
  // EDM already waits on before it takes over /planning/pos_cmd.
  finish_recheck_count_ = finish_recheck_max_;
  ROS_WARN("[FSM] stop requested by mission layer: ending exploration and handing over.");
  if (state_ != FINISH) transitState(FINISH, "StopRequest");
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;
  // Once the mission layer has called a halt, a stray trigger must not restart us. The
  // waypoint_generator node self-triggers on this same topic (see below), so this is a real
  // path, not a hypothetical one -- and re-arming here would put us back to publishing
  // /planning/pos_cmd underneath the EDM's return leg.
  if (stop_requested_) {
    ROS_WARN_THROTTLE(5.0, "[FSM] ignoring trigger: exploration was stopped by the mission.");
    return;
  }

  // A trigger is an explicit instruction to explore, so it always renews the FINISH re-check
  // budget and re-arms exploration even if the FSM has already given up.
  //
  // This matters because the FSM does not get triggered only by the mission. The
  // waypoint_generator node advertises "waypoints" relative to its own name, which resolves to
  // exactly this topic, so it self-triggers FUEL early -- on 2026-09-05 08:46 it fired 28 s
  // before the vehicle had even taken off, and the entry module's real handover trigger was
  // then silently discarded by the state_ != WAIT_TRIGGER guard below.
  finish_recheck_count_ = 0;
  if (state_ == FINISH) {
    ROS_WARN("[FSM] Trigger received in FINISH; re-arming exploration.");
    fd_->trigger_ = true;
    transitState(PLAN_TRAJ, "triggerCallback");
    return;
  }
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  Eigen::Vector3d new_pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);

  // Sanity envelope in camera_init frame, generous margin around the sdf_map box
  // (box_min/max_x=[-0.5,13.5], y=[-7,7], z=[0.2,1.9]). A corrupted FAST-LIO estimate
  // (e.g. after a physical crash degrades LiDAR scan-matching) can drift far outside the
  // arena one small step at a time -- each individual update looks plausible, so a
  // frame-to-frame jump filter alone won't catch it, but the accumulated position will
  // eventually leave this envelope. Reject those samples outright rather than feeding a
  // runaway estimate into fd_->odom_pos_, which the PLAN_TRAJ replan-seed fallback (see
  // FSMCallback) would otherwise trust unconditionally once it diverges from the predicted
  // trajectory by more than kMaxTrackingError.
  static const Eigen::Vector3d kOdomSanityMin(-3.5, -10.0, -1.0);
  static const Eigen::Vector3d kOdomSanityMax(16.5, 10.0, 3.0);
  if (fd_->have_odom_ &&
      ((new_pos.array() < kOdomSanityMin.array()).any() ||
          (new_pos.array() > kOdomSanityMax.array()).any())) {
    ROS_ERROR_THROTTLE(1.0,
        "[FSM] Rejecting implausible odometry (%.1f, %.1f, %.1f) -- outside arena sanity "
        "envelope, likely a corrupted FAST-LIO estimate. Holding last known odometry.",
        new_pos(0), new_pos(1), new_pos(2));
    return;
  }

  fd_->odom_pos_ = new_pos;

  // Odometry.twist is expressed in child_frame_id (FAST-LIO sets that to "body"), but every
  // consumer of odom_vel_ works in the world/map frame: it becomes start_vel_ for the
  // kinodynamic search, and v1 in ViewNode::computeCost where it is compared against a
  // world-frame bearing. Rotate it here rather than at any of those use sites.
  // Until 2026-09-06 FAST-LIO left twist at zero, so this read 0 unconditionally -- see the
  // comment at the publish site in laserMapping.cpp for what that cost.
  {
    const Eigen::Quaterniond q_wb(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    const Eigen::Vector3d v_world = q_wb * Eigen::Vector3d(msg->twist.twist.linear.x,
                                                           msg->twist.twist.linear.y,
                                                           msg->twist.twist.linear.z);
    fd_->odom_vel_ = v_world;
  }

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace fast_planner
