import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('messi5.jpg')
assert img is not None, "file could not be read, check with os.path.exists()"
mask = np.zeros(img.shape[:2],np.uint8)
bgdModel = np.zeros((1,65),np.float64)
fgdModel = np.zeros((1,65),np.float64)
rect = (50,50,450,290)
cv.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv.GC_INIT_WITH_RECT)
mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
img = img*mask2[:,:,np.newaxis]
plt.imshow(img),plt.colorbar(),plt.show()

# # newmask is the mask image I manually labelled
# newmask = cv.imread('newmask.png', cv.IMREAD_GRAYSCALE)
# assert newmask is not None, "file could not be read, check with os.path.exists()"
# # wherever it is marked white (sure foreground), change mask=1
# # wherever it is marked black (sure background), change mask=0
# mask[newmask == 0] = 0
# mask[newmask == 255] = 1
# mask, bgdModel, fgdModel = cv.grabCut(img,mask,None,bgdModel,fgdModel,5,cv.GC_INIT_WITH_MASK)
# mask = np.where((mask==2)|(mask==0),0,1).astype('uint8')
# img = img*mask[:,:,np.newaxis]
# plt.imshow(img),plt.colorbar(),plt.show()


image = array.line_to_2Darray(colors_crop,(new_shape[1],new_shape[0]))
mask = np.zeros(image.shape[:2],np.uint8)
f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
extractor = ForegroundExtractor(image, mask)

def mouse_click(event, x, y, flags, param):
    extractor = param[0]
    # if event == cv2.EVENT_LBUTTONDBLCLK:
    #     print("Left button double-clicked")
    #     extractor.mask[y, x] = 1

    # elif event == cv2.EVENT_RBUTTONDBLCLK:
    #     print("Right button double-clicked")
    #     extractor.mask[y, x] = 0
        
    if event == cv2.EVENT_LBUTTONDOWN:
        extractor.L_start_x, extractor.L_start_y = x, y
        extractor.L_end_x, extractor.L_end_y = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        extractor.L_end_x, extractor.L_end_y = x, y
        extractor.mask[extractor.L_start_y:extractor.L_end_y, extractor.L_start_x:extractor.L_end_x] = cv2.GC_FGD
        cv2.rectangle(extractor.image_to_show, (extractor.L_start_x, extractor.L_start_y),
                    (extractor.L_end_x, extractor.L_end_y), (0, 255, 0), 2)
        
    elif event == cv2.EVENT_RBUTTONDOWN:
        extractor.R_start_x, extractor.R_start_y = x, y
        extractor.R_end_x, extractor.R_end_y = x, y

    elif event == cv2.EVENT_RBUTTONUP:
        extractor.R_end_x, extractor.R_end_y = x, y
        extractor.mask[extractor.R_start_y:extractor.R_end_y, extractor.R_start_x:extractor.R_end_x] = cv2.GC_BGD
        cv2.rectangle(extractor.image_to_show, (extractor.R_start_x, extractor.R_start_y),
                    (extractor.R_end_x, extractor.R_end_y), (0, 0, 255), 2)   
        
        
    extractor.counter += 1
# Create a window for the display
param = [extractor]
cv2.namedWindow("Segmentation")
cv2.setMouseCallback("Segmentation", mouse_click, param)

# mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
# image = image*mask2[:,:,np.newaxis]

# class ForegroundExtractor:
#     def __init__(self, image, mask):
#         self.image = image
#         self.image_to_show = np.copy(image)
#         self.mask = mask
#         self.bgdModel = np.zeros((1,65),np.float64)
#         self.fgdModel = np.zeros((1,65),np.float64)
#         self.rect = (0,0,self.image.shape[1],self.image.shape[0])
#         self.counter = 0
#         self.L_start_x = 0
#         self.L_end_x = 0
#         self.L_start_y = 0
#         self.L_end_y = 0
        
#         self.R_start_x = 0
#         self.R_end_x = 0
#         self.R_start_y = 0
#         self.R_end_y = 0   
        
#     # def run_grabcut(self):
#     #     # bgdModel = np.zeros((1,65),np.float64)
#     #     # fgdModel = np.zeros((1,65),np.float64)
#     #     cv2.grabCut(self.image,self.mask,self.rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
#     #     return self.mask
    
    
# while True:
#     bgdModel = np.zeros((1,65),np.float64)
#     fgdModel = np.zeros((1,65),np.float64)
#     rect = (0,0,new_shape[1],new_shape[0])
#     if extractor.counter == 0:
#         cv2.grabCut(image,extractor.mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
#     else:
#         extractor.mask, extractor.bgdModel, extractor.fgdModel = cv2.grabCut(image,extractor.mask,rect,extractor.bgdModel,extractor.fgdModel,5,cv2.GC_INIT_WITH_MASK)
    
#     # gray_image = np.full((new_shape[1],new_shape[0]), 0, dtype=np.uint8)
#     # gray_image[extractor.mask == 1] = 0 # foreground
#     # gray_image[extractor.mask == 0] = 255  # background
#     # gray_image[extractor.mask == 2] = 128
#     # print(np.unique(extractor.mask))
#     mask_img = cv2.convertScaleAbs(extractor.mask)
#     cv2.normalize(mask_img, mask_img, 0, 255, cv2.NORM_MINMAX)
#     mask_img = mask_img.astype(np.uint8)
#     cv2.imshow("Mask", mask_img)
#     cv2.imshow("Segmentation", extractor.image_to_show)
#     mask = np.where((extractor.mask==2)|(extractor.mask==0),0,1).astype('uint8')
#     cv2.imshow("Image", image*mask[:,:,np.newaxis])

#     key = cv2.waitKey(1) & 0xFF

#     # Leave the programm if key 'c'
#     if key == ord("q"):
#         break
#     if key == ord("t"):
#         print("t pressed")
#         # bgdModel = np.zeros((1,65),np.float64)
#         # fgdModel = np.zeros((1,65),np.float64)
#         # rect = (0,0,new_shape[1],new_shape[0])
#         # cv2.grabCut(image,extractor.mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)
#         # ax1.imshow(extractor.mask)
#         # # ax1.colorbar()
#         # ax1.set_title('mask')
#         # mask = np.where((extractor.mask==2)|(extractor.mask==0),0,1).astype('uint8')
#         # image = image*mask[:,:,np.newaxis]
#         # ax2.imshow(image)
#         # ax2.set_title('image')
#         # plt.show()
    
