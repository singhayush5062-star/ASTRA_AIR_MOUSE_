# YOLO Model Weights Directory

Save your trained person/survivor detection model weights in this folder:

```
e:\AirmouseGCS\backend\weights\
```

### Supported Formats & File Names:
- **PyTorch format**: `best.pt`, `yolov8n.pt`, `yolov8s.pt`, `yolov11n.pt`
- **ONNX format**: `best.onnx`, `yolo.onnx`

The backend vision service will automatically scan this directory and load the model.
