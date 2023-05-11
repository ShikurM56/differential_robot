#!/usr/bin/env python3

from pyzbar import pyzbar
import cv2

cap = cv2.VideoCapture(0)


while True:

    ret, frame = cap.read()

    if ret:
        
        barcodes = pyzbar.decode(frame)
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            barcodeData = barcode.data.decode("utf-8")
            print(barcodeData)

        cv2.imshow('Codigo QR',frame)

    else:

        print('Cámara desconectada')
        break
    

    if(cv2.waitKey(1) & 0xFF == 27):
        break

cap.release()
cv2.destroyAllWindows()