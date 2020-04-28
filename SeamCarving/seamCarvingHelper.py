# -*- coding: utf-8 -*-
"""
Created on Tue Apr 28 03:22:11 2020

@author: David
"""
import cv2
import numpy as np
import scipy.signal

def energy_map(image):
    # Convert
    image = image.astype(np.float64)/255
    
    # Make kernels.
    hKernel = np.array([-1,0,1,-2,0,2,-1,0,1]).reshape(3,3)
    vKernel = np.array([-1,-2,-1,0,0,0,1,2,1]).reshape(3,3)
    
    # Make emergy map.
    if(len(image.shape)>2): # If image color.

        # Convolve image with kernels.      
        hConB = scipy.signal.convolve2d(image[:,:,0],hKernel,boundary='symm',mode='same')
        hConG = scipy.signal.convolve2d(image[:,:,1],hKernel,boundary='symm',mode='same')
        hConR = scipy.signal.convolve2d(image[:,:,2],hKernel,boundary='symm',mode='same')
        vConB = scipy.signal.convolve2d(image[:,:,0],vKernel,boundary='symm',mode='same')
        vConG = scipy.signal.convolve2d(image[:,:,1],vKernel,boundary='symm',mode='same')
        vConR = scipy.signal.convolve2d(image[:,:,2],vKernel,boundary='symm',mode='same')
        
        # Average.
        hCon = (hConB+hConG+hConR)/3
        vCon = (vConB+vConG+vConR)/3
        
        # Make energy map.
        eMap = np.absolute(hCon)+np.absolute(vCon)

        # Return.
        return eMap
    else: # If image BnW.
        # Convolve image with kernels.
        hCon = scipy.signal.convolve2d(image,hKernel,boundary='symm',mode='same')
        vCon = scipy.signal.convolve2d(image,vKernel,boundary='symm',mode='same')
        
        # Make energy map.
        eMap = np.absolute(hCon)+np.absolute(vCon)
        
        # Return.
        return eMap

#------------------------------------------------------------------------------
        
def carveNum(h,v,image,frameNum):
    # Base case.
    if ((h<1) & (v<1)):
        return image
    else:
        # Seam carved image.
        if((h<1) & (v>0)): # Just vertical seam.
            cImage, cImage2 = vCarve(image)

            for i in range(frameNum-1):
                blue3 = cImage2[:,:,0]
                blue3 = np.insert(blue3, len(blue3[0]), 0, axis=1)
                green3 = cImage2[:,:,1]
                green3 = np.insert(green3, len(green3[0]), 0, axis=1)
                red3 = cImage2[:,:,2]
                red3 = np.insert(red3, len(red3[0]), 0, axis=1)
                cImage2 = np.dstack((blue3, green3, red3))
                
            cv2.imwrite('gifFolder/remove/seamCarve_%03d.png' % frameNum, cImage2)
            if frameNum % 10 == 0:
                print 'Still working on removing seams - at frame %d' % frameNum

        elif((h>0) & (v<1)): # Just horizontal seam.
            cImage = hCarve(image)

        elif((h>0) & (v>0)): # Veritcal and horizontal seam.
            cImage = vCarve(image)
            cImage = hCarve(cImage)

        # Recursive call to function.
        return carveNum(h-1, v-1, cImage, frameNum+1)

#------------------------------------------------------------------------------

def vAdd(image, addNum):
    # Setup.
    maskArrayF = []
    maskArrayS = []
    maskArrayESort = []
    maskArraySort = []
    eImage = image

    for num in xrange(0,addNum):
        # Make energy map.
        eMap = energy_map(eImage)
        
        # Make empty arrays.
        cMapV = np.zeros([eMap.shape[0], eMap.shape[1]])
        pMapV = np.zeros([eMap.shape[0], eMap.shape[1]])
        
        # Loop for first row.
        for i in xrange(0, eMap.shape[1]):
            cMapV[0,i] = eMap[0,i]
    
        # Vertical seams.
        for row in xrange(1, eMap.shape[0]):
            for col in xrange(0, eMap.shape[1]):
                if(col == 0): # First col.
                    minP = min(cMapV[row-1,col], cMapV[row-1,col+1])
                    cMapV[row,col] = eMap[row,col] + minP
                    if(minP == cMapV[row-1,col]):
                        pMapV[row,col] = 0
                    else:
                        pMapV[row,col] = 1
                elif(col == eMap.shape[1]-1): # Last col.
                    minP = min(cMapV[row-1,col-1], cMapV[row-1,col])
                    cMapV[row,col] = eMap[row,col] + minP
                    if(minP == cMapV[row-1,col-1]):
                        pMapV[row,col] = -1
                    else:
                        pMapV[row,col] = 0
                else:
                    minP = min(cMapV[row-1,col-1], cMapV[row-1,col], cMapV[row-1,col+1])
                    cMapV[row,col] = eMap[row,col] + minP 
                    if(minP == cMapV[row-1,col-1]):
                        pMapV[row,col] = -1
                    elif(minP == cMapV[row-1,col]):
                        pMapV[row,col] = 0
                    else:
                        pMapV[row,col] = 1 
                        
        # Minimum of last row.
        lastRowV = cMapV[cMapV.shape[0]-1:]
        minIndexV = np.nonzero(lastRowV == lastRowV.min())[1][0]
        
        # Create seam mask.
        maskV = np.ones_like(cMapV,bool)
        
        # Place vertical seam path on mask.
        for i in xrange(0,maskV.shape[0]):
            maskV[(maskV.shape[0]-1)-i,minIndexV] = False
            minIndexV = (minIndexV + pMapV[(pMapV.shape[0]-1)-i,minIndexV]).astype(np.uint8)

        # Append mask.
        if(num == 0):
            maskArrayF.append(maskV)
        else:
            maskArrayS.append(maskV)
            
        # Delete seam with mask.
        if(len(eImage.shape)>2): # If image color.        
            blue = eImage[:,:,0][maskV].reshape(eImage.shape[0], eImage.shape[1]-1)
            green = eImage[:,:,1][maskV].reshape(eImage.shape[0], eImage.shape[1]-1)
            red = eImage[:,:,2][maskV].reshape(eImage.shape[0], eImage.shape[1]-1)
            
            eImage = np.dstack((blue, green, red))    
        else: # If image BnW.
            eImage = image[maskV].reshape(image.shape[0], image.shape[1]-1)

    # If only one insertion.
    if(len(maskArrayS) == 0):
        maskArraySort.append(maskArrayF[0])
    else:
        # Sort maskArrayS arrays by index.
        for i in xrange(0,maskArrayS[0].shape[1]):#each index
            for j in xrange(0,len(maskArrayS)):#each mask
                lastRow = maskArrayS[j][maskArrayS[j].shape[0]-1,:]
                index = np.nonzero(lastRow == False)[0][0]
                if(index == i):
                    maskArrayESort.append(maskArrayS[j])
        
        # Apply padding to arrays in maskArrayESort.
        for i in xrange(0,len(maskArrayESort)):
           for j in xrange(0,i):
               if(maskArrayESort[i].shape[1] != maskArrayF[0].shape[1]):
                   maskArrayESort[i] = np.insert(maskArrayESort[i],0,True,1)
           if(maskArrayESort[i].shape[1] < maskArrayF[0].shape[1]):
               for k in xrange(0,maskArrayF[0].shape[1]-maskArrayESort[i].shape[1]):
                   maskArrayESort[i] = np.insert(maskArrayESort[i],maskArrayESort[i].shape[1]-1,True,1)
        
        # Append first mask.
        maskArrayESort.append(maskArrayF[0])
        
        eImage3 = image
        
        # Combine seams on image.
        for i in xrange(0,len(maskArrayESort)):
            # Add seam.
            if(len(eImage3.shape)>2): # If color.
                blue = eImage3[:,:,0]
                green = eImage3[:,:,1]
                red = eImage3[:,:,2]
                
                # Flatten blue array.
                mFlat = maskArrayESort[i].reshape(1, eImage3.shape[0]*eImage3.shape[1])
                iFlatB = blue.reshape(1, eImage3.shape[0] * eImage3.shape[1]).astype(np.float64)
                
                # Make new array.
                eImageB = []
                
                # Append to new array.
                for i in xrange(0,iFlatB.shape[1]):
                    if(mFlat[0,i] == False):
                        eImageB.append(255)
                    else:
                        eImageB.append(iFlatB[0,i])
                        
                # Reshape.
                eImageB = np.asarray(eImageB).astype(np.uint8)
                eImageB = eImageB.reshape(eImage3.shape[0], eImage3.shape[1])
                
                # Flatten green array.
                iFlatG = green.reshape(1, eImage3.shape[0] * eImage3.shape[1]).astype(np.float64)
                
                # Make new array.
                eImageG = []
                
                # Append to new array.
                for i in xrange(0,iFlatG.shape[1]):
                    if(mFlat[0,i] == False):
                        eImageG.append(255)
                    else:
                        eImageG.append(iFlatG[0,i])
                        
                # Reshape.
                eImageG = np.asarray(eImageG).astype(np.uint8)
                eImageG = eImageG.reshape(eImage3.shape[0], eImage3.shape[1])
                
                # Flatten red array.
                iFlatR = red.reshape(1, eImage3.shape[0] * eImage3.shape[1]).astype(np.float64)
                
                # Make new array.
                eImageR = []
                
                # Append to new array.
                for i in xrange(0,iFlatR.shape[1]):
                    if(mFlat[0,i] == False):
                        eImageR.append(255)
                    else:
                        eImageR.append(iFlatR[0,i])
                        
                # Reshape.
                eImageR = np.asarray(eImageR).astype(np.uint8)
                eImageR = eImageR.reshape(eImage3.shape[0], eImage3.shape[1])
                
                eImage3 = np.dstack((eImageB, eImageG, eImageR))
            else: # If BnW.
                # Flatten arrays.
                iFlat = eImage3.reshape(1, eImage3.shape[0] * eImage3.shape[1]).astype(np.float64)
                mFlat = maskV.reshape(1, eImage3.shape[0] * eImage3.shape[1])
            
                # Make new array.
                eImageN = []
                
                # Append to new array.
                for i in xrange(0, iFlat.shape[1]):
                    if(mFlat[0,i] == False):
                        eImageN.append(255)
                    else:
                        eImageN.append(iFlat[0,i])
                        
                # Reshape.
                eImageA = np.asarray(eImageN).astype(np.uint8)
                eImage3 = eImageA.reshape(eImage3.shape[0],eImage3.shape[1])

        # Sort maskArray arrays by index.
        for i in xrange(0,maskArrayESort[0].shape[1]):#each index
            for j in xrange(0,len(maskArrayESort)):#each mask
                lastRow = maskArrayESort[j][maskArrayESort[j].shape[0]-1,:]
                index = np.nonzero(lastRow == False)[0][0]
                if(index == ((maskArrayESort[j].shape[1]-1)-i)):
                    maskArraySort.append(maskArrayESort[j])
        
        # Create padding.        
        pad = np.ones((maskArraySort[0].shape[0],1),bool)
        
        # Apply padding to arrays.
        for i in xrange(0,len(maskArraySort)):
           for j in xrange(0,i):
               maskArraySort[i] = np.append(maskArraySort[i],pad,1)

    eImage2 = image
    seamNum = 119
    frameNum = 1
    
    # Apply masks.
    for i in xrange(0,len(maskArraySort)):
        # Add seam.
        if(len(eImage2.shape)>2): # If color.
            blue = eImage2[:,:,0]
            green = eImage2[:,:,1]
            red = eImage2[:,:,2]
            
            # Flatten blue array.
            mFlat = maskArraySort[i].reshape(1, (maskArraySort[i].shape[0] * maskArraySort[i].shape[1]))
            iFlatB = blue.reshape(1, eImage2.shape[0] * eImage2.shape[1]).astype(np.float64)
            
            # Make new array.
            eImageB = []
            eImageBSeam = []
            
            # Append to new array.
            for i in xrange(0, mFlat.shape[1]):
                if(mFlat[0,i] == False):
                    eImageB.append(iFlatB[0,i])
                    eImageBSeam.append(iFlatB[0,i])
                    if(i == 0):
                        avg = (iFlatB[0,i+1])
                    elif(i == iFlatB.shape[1]-1):
                        avg = (iFlatB[0,i-1])
                    else:
                        avg = (iFlatB[0,i-1] + iFlatB[0,i+1])/2
                    eImageB.append(avg)
                    eImageBSeam.append((255))
                else:
                    eImageB.append(iFlatB[0,i])
                    eImageBSeam.append(iFlatB[0,i])
                    
            eImageB = np.asarray(eImageB).astype(np.uint8)
            eImageB = eImageB.reshape(eImage2.shape[0], eImage2.shape[1]+1)
            eImageBSeam = np.asarray(eImageBSeam).astype(np.uint8)
            eImageBSeam = eImageBSeam.reshape(eImage2.shape[0], eImage2.shape[1]+1)
            
            # Flatten green array.
            iFlatG = green.reshape(1, eImage2.shape[0] * eImage2.shape[1]).astype(np.float64)
            
            # Make new array.
            eImageG = []
            eImageGSeam = []
            
            # Append to new array.
            for i in xrange(0, mFlat.shape[1]):
                if(mFlat[0,i] == False):
                    eImageG.append(iFlatG[0,i])
                    eImageGSeam.append(iFlatG[0,i])
                    if(i == 0):
                        avg = (iFlatG[0,i+1])
                    elif(i == iFlatG.shape[1]-1):
                        avg = (iFlatG[0,i-1])
                    else:
                        avg = (iFlatG[0,i-1] + iFlatG[0,i+1])/2
                    eImageG.append(avg)
                    eImageGSeam.append((255))
                else:
                    eImageG.append(iFlatG[0,i])
                    eImageGSeam.append(iFlatG[0,i])
                    
            # Reshape.
            eImageG = np.asarray(eImageG).astype(np.uint8)
            eImageG = eImageG.reshape(eImage2.shape[0], eImage2.shape[1]+1)
            eImageGSeam = np.asarray(eImageGSeam).astype(np.uint8)
            eImageGSeam = eImageGSeam.reshape(eImage2.shape[0], eImage2.shape[1]+1)
            
            # Flatten red array.
            iFlatR = red.reshape(1, eImage2.shape[0] * eImage2.shape[1]).astype(np.float64)
            
            # Make new array.
            eImageR = []
            eImageRSeam = []
            
            # Append to new array.
            for i in xrange(0,mFlat.shape[1]):
                if(mFlat[0,i] == False):
                    eImageR.append(iFlatR[0,i])
                    eImageRSeam.append(iFlatR[0,i])
                    if(i == 0):
                        avg = (iFlatR[0,i+1])
                    elif(i == iFlatR.shape[1]-1):
                        avg = (iFlatR[0,i-1])
                    else:
                        avg = (iFlatR[0,i-1] + iFlatR[0,i+1])/2
                    eImageR.append(avg)
                    eImageRSeam.append((255))
                else:
                    eImageR.append(iFlatR[0,i])
                    eImageRSeam.append(iFlatR[0,i])
                    
            # Reshape.
            eImageR = np.asarray(eImageR).astype(np.uint8)
            eImageR = eImageR.reshape(eImage2.shape[0], eImage2.shape[1]+1)
            eImageRSeam = np.asarray(eImageRSeam).astype(np.uint8)
            eImageRSeam = eImageRSeam.reshape(eImage2.shape[0],eImage2.shape[1]+1)
            
            eImage2 = np.dstack((eImageB, eImageG, eImageR))
            eImage2Seam = np.dstack((eImageBSeam, eImageGSeam, eImageRSeam))
            
            for i in range(seamNum-1):
                blue3 = eImage2Seam[:,:,0]
                blue3 = np.insert(blue3, len(blue3[0]), 0, axis=1)
                green3 = eImage2Seam[:,:,1]
                green3 = np.insert(green3, len(green3[0]), 0, axis=1)
                red3 = eImage2Seam[:,:,2]
                red3 = np.insert(red3, len(red3[0]), 0, axis=1)
                eImage2Seam = np.dstack((blue3, green3, red3))
                
            cv2.imwrite('gifFolder/add/seamAdd_%03d.png' % frameNum, eImage2Seam)
            frameNum += 1
            seamNum -= 1
            
            if frameNum % 20 == 0:
                print 'Still working on Adding seams - at frame %d' % frameNum
                
        else: # If BnW.
            # Flatten arrays.
            iFlat = image.reshape(1, eImage2.shape[0] * eImage2.shape[1]).astype(np.float64)
            mFlat = maskArraySort[i].reshape(1,maskArraySort[i].shape[0] * maskArraySort[i].shape[1])
        
            # Make new array.
            eImageN = []
            
            # Append to new array.
            for i in xrange(0,mFlat.shape[1]):
                if(mFlat[0,i] == False):
                    eImageN.append(iFlat[0,i])
                    if(i == 0):
                        avg = (iFlat[0,i+1])
                    elif(i == iFlat.shape[1]-1):
                        avg = (iFlat[0,i-1])
                    else:
                        avg = (iFlat[0,i-1]+iFlat[0,i+1])/2
                    eImageN.append(avg)
                else:
                    eImageN.append(iFlat[0,i])
                    
            # Reshape.
            eImageA = np.asarray(eImageN).astype(np.uint8)
            eImage2 = eImageA.reshape(eImage2.shape[0], eImage2.shape[1]+1)
    
    # Return.
    return eImage2

#------------------------------------------------------------------------------
    
def vCarve(image):
    # Make energy map.
    eMap = energy_map(image)
    
    # Make empty arrays.
    cMapV = np.zeros([eMap.shape[0], eMap.shape[1]])
    pMapV = np.zeros([eMap.shape[0], eMap.shape[1]])
    
    # Loop for first row.
    for i in xrange(0, eMap.shape[1]):
        cMapV[0,i] = eMap[0,i]

    # Vertical seams.
    for row in xrange(1,eMap.shape[0]):
        for col in xrange(0,eMap.shape[1]):
            if(col == 0): # First col.
                minP = min(cMapV[row-1,col], cMapV[row-1,col+1])
                cMapV[row,col] = eMap[row,col] + minP
                if(minP == cMapV[row-1,col]):
                    pMapV[row,col] = 0
                else:
                    pMapV[row,col] = 1
            elif(col == eMap.shape[1]-1): # Last col.
                minP = min(cMapV[row-1,col-1], cMapV[row-1,col])
                cMapV[row,col] = eMap[row,col] + minP
                if(minP == cMapV[row-1,col-1]):
                    pMapV[row,col] = -1
                else:
                    pMapV[row,col] = 0
            else:
                minP = min(cMapV[row-1,col-1], cMapV[row-1,col], cMapV[row-1,col+1])
                cMapV[row,col] = eMap[row,col] + minP
                if(minP == cMapV[row-1,col-1]):
                    pMapV[row,col] = -1
                elif(minP == cMapV[row-1,col]):
                    pMapV[row,col] = 0
                else:
                    pMapV[row,col] = 1
    
    # Minimum of last row.
    lastRowV = cMapV[cMapV.shape[0]-1:]
    minIndexV = np.nonzero(lastRowV == lastRowV.min())[1][0]
    
    # Create seam mask.
    maskV = np.ones_like(cMapV,bool)
    
    # Place vertical seam path on mask.
    for i in xrange(0,cMapV.shape[0]):
        maskV[(maskV.shape[0]-1)-i,minIndexV]=False
        minIndexV = (minIndexV + pMapV[(pMapV.shape[0]-1)-i,minIndexV]).astype(np.uint8)   
    
    blue2 = image[:,:,0]
    blue2[np.invert(maskV)] = 255
    green2 = image[:,:,1]
    green2[np.invert(maskV)] = 255
    red2 = image[:,:,2]
    red2[np.invert(maskV)] = 255
    image2 = np.dstack((blue2,green2,red2))

    # Delete seam with mask.
    if(len(image.shape)>2): # If image color.        
        blue = image[:,:,0][maskV].reshape(image.shape[0], image.shape[1]-1)
        green = image[:,:,1][maskV].reshape(image.shape[0], image.shape[1]-1)
        red = image[:,:,2][maskV].reshape(image.shape[0], image.shape[1]-1)
        
        image = np.dstack((blue,green,red))    
    else: # If image BnW.
        image = image[maskV].reshape(image.shape[0], image.shape[1]-1)

    # Return.
    return image, image2

#--------------------------------------------------------------------------------------------------

def hCarve(image):
    # Make energy map.
    eMap = energy_map(image)
    
    # Make empty arrays.
    cMapH = np.zeros([eMap.shape[0], eMap.shape[1]])
    pMapH = np.zeros([eMap.shape[0], eMap.shape[1]])
    
    # Loop for first col.
    for i in xrange(0,eMap.shape[0]):
        cMapH[i,0] = eMap[i,0]

    # Horizontal seams.
    for col in xrange(1,eMap.shape[1]):
        for row in xrange(0,eMap.shape[0]):
            if(row == 0): # First row.
                minP = min(cMapH[row,col-1], cMapH[row+1,col-1])
                cMapH[row,col] = eMap[row,col] + minP
                if(minP == cMapH[row,col-1]):
                    pMapH[row,col] = 0
                else:
                    pMapH[row,col] = 1
            elif(row == eMap.shape[0]-1): # Last row.
                minP = min(cMapH[row-1,col-1], cMapH[row,col-1])
                cMapH[row,col] = eMap[row,col] + minP
                if(minP == cMapH[row-1,col-1]):
                    pMapH[row,col] = -1
                else:
                    pMapH[row,col] = 0
            else:
                minP = min(cMapH[row-1,col-1], cMapH[row,col-1], cMapH[row+1,col-1])
                cMapH[row,col] = eMap[row,col] + minP
                if(minP == cMapH[row-1,col-1]):
                    pMapH[row,col] = -1
                elif(minP == cMapH[row,col-1]):
                    pMapH[row,col] = 0
                else:
                    pMapH[row,col] = 1  

    # Minimum of last row.
    lastColH = cMapH[:,cMapH.shape[1]-1]
    minIndexH = np.nonzero(lastColH == lastColH.min())[0][0]

    # Create seam mask.
    maskH = np.ones_like(cMapH,bool)

    # Place horizontal seam path on mask.   
    for i in xrange(0,cMapH.shape[1]):
        maskH[minIndexH,(maskH.shape[1]-1)-i] = False
        minIndexH = (minIndexH + pMapH[minIndexH, (pMapH.shape[1]-1)-i]).astype(np.uint8)
    
    # Rotate 90 degrees.
    maskH = np.rot90(maskH)
    image = np.rot90(image)
    
    # Delete seam with mask.
    if(len(image.shape)>2): # If image color.     
        blue = image[:,:,0][maskH].reshape(image.shape[0], image.shape[1]-1)
        green = image[:,:,1][maskH].reshape(image.shape[0], image.shape[1]-1)
        red = image[:,:,2][maskH].reshape(image.shape[0], image.shape[1]-1)
        image = np.dstack((blue,green,red))
        image = np.rot90(image,3)
    else: # If image BnW.
        image = image[maskH].reshape(image.shape[0], image.shape[1]-1)
        image = np.rot90(image, 3)

    # Return.
    return image