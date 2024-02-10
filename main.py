import cv2
import numpy as np
#function to find the average of a list
def average(list):
   return(int(sum(list)/len(list)))

#get video from camera
video = cv2.VideoCapture(0)

while True:
  #get every frame of video
  ret, frame = video.read()
  #breaks if the video has ended
  if frame is None:
      break
  #duplicate the frame
  dup = frame.copy()

  #turns into grayscale
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  #gaussianblur
  gray = cv2.GaussianBlur(gray,(15,15),0)
  #finds the black areas
  darkmask = cv2.inRange(gray, 10, 55)
  #isolates the darker areas
  maskedimg = cv2.bitwise_and(gray,darkmask)
  # gets all the edges
  canny = cv2.Canny(maskedimg, 140, 150)

  #gets the  height and width of the frame
  height, width = canny.shape
  #this makes a rectangle
  mask = np.zeros_like(gray)
  #points of the rectangle
  point1 = (100,height-150)
  point2 = (100,100)
  point3 = (height-150,100)
  point4 = (height-150,height-150)
  #array showing the coordinates of the trapezoid
  trapezoid = np.array([[point1, point2, point3, point4]])
  #outlines the rectangle for user use
  cv2.line(frame, point1, point2, (255, 255, 255), 3)
  cv2.line(frame, point2, point3, (255, 255, 255), 3)
  cv2.line(frame, point3, point4, (255, 255, 255), 3)
  cv2.line(frame, point4, point1, (255, 255, 255), 3)
  #creates plolygon
  mask = cv2.fillPoly(mask, trapezoid, 255)
  #adds the mask
  mask = cv2.bitwise_and(canny, mask)


  #houghlines (gets a set of lines that are outlined in the masked image)
  lines = cv2.HoughLinesP(mask, 1, np.pi/180, 50, maxLineGap =100, minLineLength= 80)
  #lists for slopes
  slopes = {}
  #keeps track of the number of kinda unique slopes
  slopecount = {}
  #makes sure there are lines
  if lines is not None:
    for line in lines:
        #gets endpoints for line
        x1,y1,x2,y2 = line[0]
        #if the slope is infinite then set it to a huge number
        try:
            slope = (y1-y2)/(x1-x2)
        except:
            slope = 99999999999999999999999
        #adds the slope to the slopes dictionary with the endpoints of the line
        slopes[(x1,y1),(x2,y2)] = slope
        #keeps track of the unique slope count
        test = 0
        for a in slopecount:
           if a -0.2 < slope <0.2 +a:
              slopecount[a] += 1
              test = 1
              break
        if test == 0:
           slopecount[slope] = 1

  #if dictionary is no empty
  if slopes !={}:
       #finding the most common slope (slope of the parallel lines)
       sloper = 239018
       for a in slopecount:
          if sloper == 239018:
             sloper = a
          else:
             if slopecount[sloper] < slopecount[a]:
                sloper = a
       #lists to keep track of the x,y values and yintercepts of the lines that fit the slope
       pointsy = {}
       yintercepts = []
       for a in slopes:
          if sloper- 0.1< slopes[a]<sloper+0.1:
              points = a

              yinterception = points[0][1]-sloper*points[0][0]
              checker = 0
              for b in pointsy:
                  if b - 20 < yinterception < b+20:
                      xval = pointsy[b][0]
                      yval = pointsy[b][1]
                      xval.append(points[0][0])
                      xval.append(points[1][0])
                      yval.append(points[0][1])
                      yval.append(points[1][1])
                      pointsy[b] = [xval,yval]
                      checker = 1
                      break
              if checker == 0:
                  xval = [points[0][0],points[1][0]]
                  yval = [points[0][1],points[1][1]]
                  pointsy[yinterception] = [xval,yval]

              testing =0
              for yintercept in yintercepts:
                  if yintercept - 1 < yinterception < yintercept +1:
                      testing = 1
              if testing == 0:
                  yintercepts.append(yinterception)


       if pointsy != {}:
           for intercepty in pointsy:
               xval = pointsy[intercepty][0]
               yval = pointsy[intercepty][1]
               if sloper < 0:
                   cv2.line(frame, (min(xval),max(yval)), (max(xval),min(yval)), (255, 25, 120), 10)
               else:
                   cv2.line(frame, (min(xval),min(yval)), (max(xval),max(yval)), (255, 25, 120), 10)

       #sees if the yintercept list is none
       if yintercepts != []:
           print(yintercepts)
           #average of the yintercepts
           averagey = (min(yintercepts) + max(yintercepts))/2
           #draws the midline
           cv2.line(frame, (0, int(averagey)), (width, int(width*sloper+averagey)), (208, 224, 64), 3)



  #shows the untouched video
  cv2.imshow("Camera", dup)
  #video with lines
  cv2.imshow("Lines", frame)


  #breaks the video if chracter i is pressed
  if cv2.waitKey(15) & 0xFF == ord('i'):
    break
video.release()
cv2.destroyAllWindows()





