from ultralytics import YOLO

model = YOLO('yolov12n.yaml')

# Train the model
results = model.train(
  data='/root/autodl-tmp/rmdata/data.yaml',
  epochs=600, 
  batch=96, 
  imgsz=640,
  scale=0.5,  # S:0.9; M:0.9; L:0.9; X:0.9
  mosaic=1.0,
  mixup=0.0,  # S:0.05; M:0.15; L:0.15; X:0.2
  copy_paste=0.1,  # S:0.15; M:0.4; L:0.5; X:0.6
  device="0",
)

# Evaluate model performance on the validation set
metrics = model.val()

# Perform object detection on an image
results = model("/root/autodl-tmp/rmdata/test/000018.jpg")
results[0].show()
