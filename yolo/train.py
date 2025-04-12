from ultralytics import YOLO

model = YOLO("yolo11s.pt")  # Or "yolov11.pt" if you have the full model
model.train(
    data="coin_data.yaml",
    epochs=60,
    imgsz=640,
    batch=8,
    name="train15"
)`
