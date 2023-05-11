import cv2 # opencv
import sys # Parámetros y funciones específicos del sistema


def getPosition(event, x, y, flags, param):

	global coordinate

	if event == cv2.EVENT_LBUTTONDOWN:
		coordinate = [(x, y)]
		print(coordinate)
		
		# frame.shape[1]  coordinate x width
		# frame.shape[0]   coordinate y high

		##################### CAMARA EN MANO #################
		
		# xrobot = frame.shape[1]/2 - x 
		# yrobot = frame.shape[0]/2 - y

		
		##################### CAMARA FIJA #################

		xrobot = x - frame.shape[1]/2
		yrobot = frame.shape[0]/2 - y
		
		print(f" xrobot = {xrobot}")
		print(f" yrobot = {yrobot}")


cap = cv2.VideoCapture(0) # Crear objeto VideoCapture

coordinate = []

if cap.isOpened():
    print("USB Cam initializatized")
else:
    print("USB Cam disconnected")
    sys.exit(1)

winName = 'USB_CAM'
cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback(winName, getPosition)


while(True):
    ret, frame = cap.read()# Captura de frame

    
    if ret:
        cv2.circle(frame,(int(frame.shape[1]/2),int(frame.shape[0]/2)),20, (0,0,255),3)
        cv2.imshow(winName,frame)
    else:
        print("Usb Cam disconnected")
        break
       
    if (cv2.waitKey(10) & 0xFF == 27):
        break
    
cap.release()
cv2.destroyAllWindows()




     
    

