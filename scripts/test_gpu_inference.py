#!/usr/bin/env python3
"""GPU + inference smoke test for the NIDAR Docker container.

Confirms the container can see the host GPU through the NVIDIA Container
Toolkit and can run a real forward pass (YOLOv8n) on it. Run inside the
container after `docker run --gpus all ...`:

    python3 scripts/test_gpu_inference.py
"""
import sys
import time

import numpy as np
import torch


def main() -> int:
    print(f"torch version: {torch.__version__}")
    cuda_available = torch.cuda.is_available()
    print(f"CUDA available: {cuda_available}")

    if not cuda_available:
        print("FAIL: no GPU visible to the container. Check that the host has "
              "nvidia-container-toolkit installed and the container was launched "
              "with --gpus all (see scripts/docker_dev_start.sh).")
        return 1

    device_name = torch.cuda.get_device_name(0)
    print(f"CUDA device: {device_name}")
    print(f"CUDA capability: {torch.cuda.get_device_capability(0)}")

    from ultralytics import YOLO

    print("Loading yolov8n.pt (downloads on first run)...")
    model = YOLO("yolov8n.pt")
    model.to("cuda")

    dummy_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    start = time.time()
    results = model.predict(dummy_frame, device=0, verbose=False)
    elapsed = time.time() - start

    print(f"Inference OK: {len(results)} result(s) in {elapsed * 1000:.1f} ms on {device_name}")
    print("PASS: GPU inference pipeline is working inside the container.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
