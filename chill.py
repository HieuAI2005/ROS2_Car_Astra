import tensorflow as tf
from tensorflow.keras import layers, models
import cv2 
import numpy as np
import matplotlib.pyplot as plt 

def build_unet(input_shape=(128, 128, 3)):
    inputs = layers.Input(input_shape)

    c1 = layers.Conv2D(8, 3, activation='relu', padding='same')(inputs)
    c1 = layers.Conv2D(8, 3, activation='relu', padding='same')(c1)
    p1 = layers.MaxPooling2D()(c1)

    c2 = layers.Conv2D(16, 3, activation='relu', padding='same')(p1)
    c2 = layers.Conv2D(16, 3, activation='relu', padding='same')(c2)
    p2 = layers.MaxPooling2D()(c2)

    c3 = layers.Conv2D(32, 3, activation='relu', padding='same')(p2)
    c3 = layers.Conv2D(32, 3, activation='relu', padding='same')(c3)

    u1 = layers.UpSampling2D()(c3)
    concat1 = layers.Concatenate()([u1, c2])
    c4 = layers.Conv2D(16, 3, activation='relu', padding='same')(concat1)
    c4 = layers.Conv2D(16, 3, activation='relu', padding='same')(c4)

    u2 = layers.UpSampling2D()(c4)
    concat2 = layers.Concatenate()([u2, c1])
    c5 = layers.Conv2D(8, 3, activation='relu', padding='same')(concat2)
    c5 = layers.Conv2D(8, 3, activation='relu', padding='same')(c5)

    outputs = layers.Conv2D(1, 1, activation='sigmoid')(c5)

    model = models.Model(inputs, outputs)
    return model

model = build_unet()

model.compile(
    optimizer='adam',
    loss='binary_crossentropy',
    metrics=['accuracy', tf.keras.metrics.MeanIoU(num_classes=2)]
)

img = cv2.imread('/kaggle/input/123123/3.jpg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_resized = cv2.resize(img, (128, 128))

# Tiền xử lý ảnh
img_input = img_resized / 255.0
img_input = np.expand_dims(img_input, axis=0)

# Dự đoán mask
pred = model.predict(img_input)[0] 
confidence = np.max(pred)        
binary_mask = (pred > 0.5).astype('uint8')

# Tạo overlay màu đỏ
overlay = img_resized.copy()
overlay[binary_mask.squeeze() == 1] = [255, 0, 0]  

# Làm mờ vùng mask lên ảnh gốc
alpha = 0.5
blended = cv2.addWeighted(img_resized, 1 - alpha, overlay, alpha, 0)

# Vẽ label + confidence
label_text = f"Lane: {confidence:.2f}"

# Hiển thị kết quả
plt.figure(figsize=(6, 6))
plt.imshow(blended)
plt.axis('off')
plt.title("Prediction Overlay with Label")
plt.show()