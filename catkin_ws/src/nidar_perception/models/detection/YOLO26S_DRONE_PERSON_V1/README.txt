YOLO26S DRONE PERSON V1

Model: YOLO26s (Ultralytics), fine-tuned from yolo26s.pt
Class: person (single class)
Image size: 640x640
Training: 100 epochs, batch 4, FINAL_PERSON_DRONE_DATASET (drone-view people);
          see args.yaml and results.csv (per-epoch metrics). No vertical-flip augmentation.
Source: yolo26s_drone_person_v1-20261003T094021Z-1-001.zip (training run
        yolo26s_drone_person_v1). Only best.pt, args.yaml and results.csv are shipped here;
        last.pt, best_int8.onnx and the training plots stay in the zip.

Validation (best epoch 94, on the drone-view set):
Precision: 0.729
Recall: 0.521
mAP50: 0.583
mAP50-95: 0.306
These come from a harder dataset than PERSON_DETECTION_MODEL_V3's and are not comparable
with its numbers.

Runtime: nidar_perception/launch/detector.launch (model_path, rotate_180=true because the sim
camera is mounted rolled 180 deg and this model only detects upright people).

Sim check 2026-10-03 (158 frames of a survivor, 2.5-4 m): person found in 146 de-rotated
frames vs 0 raw (upside-down) frames at confidence >= 0.55; PERSON_DETECTION_MODEL_V3 76 vs 0.
