import cv2 as cv
import numpy as np
import argparse
import time
import math
import serial

arduino = serial.Serial(port = 'COM4', baudrate = 115200, timeout = .1)
time.sleep(1)

tn = time.time()
tElapsed = 0

prevCommandX,prevCommandY = 101,360

x1,x2,x3,y1,y2,y3=0,0,0,0,0,0
movingStationary = False#is the robot moving to hit a stationary puck

def rotate_image(mat, angle):
    """
    Rotates an image (angle in degrees) and expands image to avoid cropping
    """
    
    height, width = mat.shape[:2] # image shape has 3 dimensions
    image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape
    
    rotation_mat = cv.getRotationMatrix2D(image_center, angle, 1.)
    
    # rotation calculates the cos and sin, taking absolutes of those.
    abs_cos = abs(rotation_mat[0,0]) 
    abs_sin = abs(rotation_mat[0,1])
    
    # find the new width and height bounds
    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)
    
    # subtract old image center (bringing image back to origo) and adding the new image center coordinates
    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]
    
    # rotate image with the new bounds and translated rotation matrix
    rotated_mat = cv.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    return rotated_mat

def onMouse(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
       # draw circle here (etc...)
       hsvPixel = green[y, x]
       h = hsvPixel[0]   # Convert to 0-360° range
       s = hsvPixel[1]   # Convert to percentage
       v = hsvPixel[2] 
       print('x = %d, y = %d'%(x, y))
       #print('h = %d, s = %d, v = %d'%(h,s,v))
       
def detectPuck():
  #find the puck
  #find a streak of green pixels
  gStreak = 0
  br = 0
  xi, yi = 0,0
  xSum , ySum = 0,0
  #xf, yf = x,y  #set x and y as previous, in case no new puck location is detected
  n = 0
  x,y = 0,0
  xf,yf = 0,0

  #find general puck location, by finding 4 green pixels in a row
  for y in range(60,650,20):
    for x in range(45,1250,8):
        
      hsvPixel = green[y, x] 
      s = hsvPixel[1]

      green [y,x] = (0,0,255)

      if s>40:
        gStreak +=1
      else:
        gStreak = 0
      if gStreak >=3:
        br = 1
        xi = x
        yi = y
        break
            
    if(br == 1):
      br = 0
      break

  #find a more accurate puck position
  if gStreak>=3:
    for x in range(xi-80,xi+80,1):
      for y in range(yi-60,yi+90,1):
        if x>0 and y>0 and x<1280 and y<720:
          hsvPixel = green[y, x]
          s = hsvPixel[1]
          #frame[y,x] = (0,255,0)  #draw dots to know where looked
          if s> 90 and y>45 and y<670 and x>10 and x<1265:  #change this to make detection more accurate at edges
            n += 1
            xSum = xSum + x
            ySum = ySum + y
    if n>0:
      xf = int(xSum/n)
      yf = int(ySum/n)
      cv.circle(green,(xf,yf),30,(255,0,0),5)
      cv.circle(frame,(xf,yf),30,(255,0,0),5)

  global x1,y1
  x1,y1 = xf,yf
  return (xf,yf)

def lineEnd(xi,yi,ddx,ddy): #this function gives us the coords where puck hits wall givein inital coords and speeds
  xf,yf = 0,0
  yU,yD,xL,xR =80,614,66,1222 
  # which wall does it bounce?
  wallHit = 'n'

  if ddx==0:              # if not moving in x direction
    if ddy>0:
      xf,yf= xi,610
      wallHit = 'b'
    if ddy<0:
      xf,yf = xi,83
      wallHit = 't'
  elif ddy==0:            # if not moving in y direction
    if ddx>0:
      xf,yf= 1222,yi
      wallHit = 'r'
    if ddx<0:
      xf,yf = 71,yi
      wallHit = 'l'
  elif ddx>0 and ddy>0:   # down and to right
    potXf = ((yD-yi)/(ddy/ddx))+xi
    potYf = ((xR-xi)*(ddy/ddx)) +yi
    if potXf>=71 and potXf <=1222: #will hit the bottom side
      xf,yf = potXf, 610
      wallHit = 'b'
    else:                          #will hit right side
      xf,yf = 1222, potYf 
      wallHit = 'r'
  elif ddx<0 and ddy>0:   # down and to left
    potXf = ((yD-yi)/(ddy/ddx))+xi   
    potYf = ((xL-xi)*(ddy/ddx)) +yi  
    if potXf>=71 and potXf <=1222: #will hit the bottom edge
      xf,yf = potXf, 610
      wallHit = 'b'
    else:
      xf,yf = 71, potYf 
      wallHit = 'l'
  elif ddx>0 and ddy<0:   # up and to right
    potXf = ((yU-yi)/(ddy/ddx))+xi
    potYf = ((xR-xi)*(ddy/ddx)) +yi
    if potXf>=71 and potXf <=1222: #will hit the top side
      xf,yf = potXf, 83
      wallHit = 't'
    else:                          #will hit right side
      xf,yf = 1222, potYf 
      wallHit = 'r'
  elif ddx<0 and ddy<0:   # up and to left
    potXf = ((yU-yi)/(ddy/ddx))+xi
    potYf = ((xL-xi)*(ddy/ddx)) +yi
    if potXf>=71 and potXf <=1222: #will hit the top edge
      xf,yf = potXf, 83
      wallHit = 't'
    else:
      xf,yf = 71, potYf #will hit left edge
      wallHit = 'l'
  return(int(xf),int(yf),wallHit)

def predict(x1,y1,x2,y2,x3,y3,dt): #pddx and pddy are previous speeds, used for averaging
  #1 is current coords, 2 is from 1 frame ago, 3 is from 2 frames ago
  goodVal = True
  ddx,ddy,ddxPrev,ddyPrev = 0,0,0,0
  sddx,sddy = ddx,ddy #smoothed speed
  BCddx,BCddy = sddx,sddy
  dx, dy = x1-x2 , y1-y2
  dxPrev, dyPrev = x2-x3 , y2-y3
  
  perpBounceCoeff = .61
  parBounceCoeff = .74
  bounceCoeff = .81  #coefficient of restitution

  if(dt != 0 ):
    ddx = dx/dt         #velocity given the last two spots
    ddy = dy/dt
    ddxPrev,ddyPrev = dxPrev/dt, dyPrev/dt
    #sddx,sddy = ddx,ddy  
    sddx,sddy = int((ddx+ddxPrev)/2), int((ddy+ddyPrev)/2) 
               
  else:
    goodVal = False

  if(x1==0 or x2==0 or y1==0 or y2==0):
    goodVal = False

  
  Bcoords = lineEnd(x1,y1,sddx,sddy)
  #find the second line BC
  if (Bcoords[2]=='t' or Bcoords[2]=='b'):
    BCddx,BCddy = sddx*parBounceCoeff, -1*sddy*perpBounceCoeff
  elif (Bcoords[2]=='l' or Bcoords[2]=='r'):
    BCddx,BCddy = -1*sddx*perpBounceCoeff, sddy*parBounceCoeff

  Ccoords = lineEnd(Bcoords[0],Bcoords[1],BCddx,BCddy)

  #makes sure past 3 frames, puck moving same direction, prevent steady state errors
  if (dx>0 and dxPrev<=0)or(dx<0 and dxPrev>=0) or (dy>0 and dyPrev<=0)or(dy<0 and dyPrev>=0):
    goodVal = False
  
  if (abs(ddx)+abs(ddy))<175:
    goodVal = False

  if goodVal == True:
    #AB line
    cv.line(frame,(x1,y1),(Bcoords[0],Bcoords[1]),(128,128,0),4)
    cv.line(green,(x1,y1),(Bcoords[0],Bcoords[1]),(128,128,0),4)
    #BC line
    cv.line(frame,(Bcoords[0],Bcoords[1]),(Ccoords[0],Ccoords[1]),(128,128,0),4)
    cv.line(green,(Bcoords[0],Bcoords[1]),(Ccoords[0],Ccoords[1]),(128,128,0),4)

  #print (sddx,"    ",sddy)
  return (sddx,sddy,BCddx,BCddy,Bcoords[0],Bcoords[1],Ccoords[0],Ccoords[1], Bcoords[2],Ccoords[2],goodVal,ddxPrev,ddyPrev)
 
def controlRobot(x1,y1,xB,yB,xC,yC, ddx1,ddy1,ddx2,ddy2,prevCommandX,prevCommandY):   #add goodval check
  #x1y1 = current position x2y2 is first bounce x3y3 is second bounce
  #ddx1/y1 is speed before bounce, ddx2/y2 is speed after bounce
  #also resolve distance from center to center to edge to edge and time delays
  commandX, commandY,tToI = 101,360,0
  robotDist = 0
  robotVelocityAvg = 2000 #constant, average of robot speed
  #robotVelocityAvg = 2800
  robotTimeNeeded = 0
  timeToIntercept =90000
  yAtIntercept = 0
  timeBuffer = .15  #buffer, increasing increases the amount of extra time the robot needs
    
  allPotX=[0]   #this will be an array of all viable x and y intercepts
  allPotY=[0]
  allTtoI = [0]

  furthestX = x1  #finds the furthest distance towards the goal the puck will travel on current trajectory.
  if(xB<x1):
    furthestX = xB
  if(xC<xB):
    furthestX = xC
    
  if goodVal:  
    cv.line(frame,(furthestX,70),(furthestX,650),(255,0,0),4)

    if(ddx1>=0):  #if puck going away, dont move robot
      commandX,commandY = 101,360
    else:         #if puck moving towards us,
      
      for potXIntercept in range (min(600,x1),furthestX,-10):   #loop through midway to closest x will reach
          
        #we need to find if this is a viable option
        if(potXIntercept>=xB):            #if this intercept is before the bounce, we can find the time as such
          timeToIntercept = abs((potXIntercept-x1)/ddx1)
          yAtIntercept = int(y1 + (timeToIntercept*ddy1))
        else:
          tToB = abs((xB-x1)/ddx1) #time to go from current pos to B (first bounce)
          timeToIntercept = tToB + abs((potXIntercept-xB)/ddx2) # time is time from now till first bounce, plus time from bounce to intercept 
          yAtIntercept = int(yB + ddy2*(timeToIntercept-tToB))

        robotDist = math.sqrt((potXIntercept-60)**2 + (yAtIntercept-360)**2) #find distance from robot
        robotTimeNeeded = robotDist/robotVelocityAvg  # time the robot would need to hit the puck 

        if (robotTimeNeeded + timeBuffer < timeToIntercept): #this is possible
          commandX,commandY = potXIntercept,yAtIntercept
          cv.circle(frame,(potXIntercept,yAtIntercept),5,(250,250,0),5)
          allPotX.append(int(potXIntercept))
          allPotY.append(int(yAtIntercept))
          allTtoI.append(timeToIntercept)
        else:
          cv.circle(frame,(potXIntercept,yAtIntercept),5,(0,250,250),5)

      #this is the midpoint of all potential intercepts  
      midX,midY,midTtoI = allPotX[int(len(allPotX)/5)], allPotY[int(len(allPotY)/5)],allTtoI[int(len(allTtoI)/5)]

      #find distance from prevCommandY and y revaluated at the same x , as well as new time to Intercept
      if prevCommandX>=xB:
        revaluatedTtoI = abs((prevCommandX-x1)/ddx1)
        yRevaluated = int(y1 + (revaluatedTtoI*ddy1))
      elif prevCommandX<xB:
        revaluatedTtoI = abs((xB-x1)/ddx1) + abs((prevCommandX-xB)/ddx2)
        yRevaluated =  int(yB + ddy2*abs((prevCommandX-xB)/ddx2))

      yRevaluatedDist = abs(prevCommandY-yRevaluated)

      newcommand = 0

      if prevCommandX == 101 or yRevaluatedDist>100 or x1<prevCommandX:  #if last command was origin, midway is our new command
        commandX,commandY,tToI = midX,midY,midTtoI
        newcommand =1

      elif prevCommandX != 101 and yRevaluatedDist<100: # if last command was close, keep x and revaluate commandy
        commandX = prevCommandX
        commandY = yRevaluated
        tToI = revaluatedTtoI
        newcommand = 2
      
      #cv.putText(frame, f'commandX: {commandX:.2f}', (200, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (120, 0, 255), 2)

      robotDist = math.sqrt((commandX-60)**2 + (commandY-360)**2) #find distance from robot
      robotTimeNeeded = robotDist/robotVelocityAvg

  if commandX == 0 or commandY ==0:
    commandX,commandY = 101,360

  go = 0 # should the robot go or not
  if robotTimeNeeded + timeBuffer>= tToI : # if robot is on course or slow, make it go
    go = 1

  cv.circle(frame,(commandX,commandY),15,(0,0,250),15)
  #now that we have the options, determine the command coordinates
  return (commandX,commandY,go,tToI,robotTimeNeeded)
        



# define a video capture object 
vid = cv.VideoCapture(2,cv.CAP_DSHOW) 
vid.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
vid.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

lower_green = np.array([25, 50, 10])
upper_green = np.array([95, 255, 255])


while(True): 
    
  deltaT = (time.time() - tn)-tElapsed
  tElapsed = time.time()-tn

  #print (deltaT)
  # Capture the video frame 
  ret, frame = vid.read() 
  frame = rotate_image(frame,-180.6)
  hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)        #convert from bgr to hsv
  mask = cv.inRange(hsv, lower_green, upper_green)  #filter non green pixels
  green = cv.bitwise_and(hsv, hsv, mask=mask)      #apply mask to only show green

  detectPuck()
  if x1 == 0 or y1 == 0:  #incase cant find puck, make it the last known location
    x1,y1 = x2,y2

  coords = predict(x1,y1,x2,y2,x3,y3,deltaT)

  BX,BY = coords[4],coords[5]
  CX,CY = coords[6],coords[7]
  goodVal = coords[10]
  sddx,sddy = coords[0],coords[1]
  ddxPrev,ddyPrev = coords[11],coords[12]

  commandCoords = controlRobot(x1,y1,BX,BY,CX,CY,coords[0],coords[1],coords[2],coords[3],prevCommandX,prevCommandY)
  commandX,commandY,go = commandCoords[0], commandCoords[1],commandCoords[2]  

  prevCommandX,prevCommandY = commandX,commandY

  x3,y3 = x2,y2
  x2,y2 = x1,y1

  if go==1:# if we should start moving the robot
    cv.line(frame,(101,360),(commandX,commandY),(250,250,250),5)# draw line of robot trajectory
  if go == 0:
    commandX,commandY = 101,360

  

  #hit puck if its slow moving in robot's territory
  if (x1<590) and (abs(sddx)+abs(sddy))<800 and (abs(ddxPrev)+abs(ddyPrev))<800:
    if (abs(sddx)+abs(sddy))<400 and (abs(ddxPrev)+abs(ddyPrev))<400:  # if moving to stationary puck, give extra leeway
      movingStationary = True
      commandX,commandY = x1,y1
    if movingStationary:
      commandX,commandY = x1,y1
  else:
    movingStationary = False

  #print speed on screen
  speed = abs(sddx)+abs(sddy)
  cv.putText(frame, f'Speed: {speed:.2f}', (50, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
  #cv.putText(frame, f'goodval: {goodVal}', (540, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

  #print(commandX)

  #send commands to arduino
  ardX = (commandY-360) - 5000
  ardY = commandX-85
  if ardY<10:
    ardY = 10
  ardX,ardY = str(ardX),str(ardY)
  ardX,ardY = ardX+'\r', ardY+'\r'
  arduino.write(str.encode(ardX))
  time.sleep(.009)
  arduino.write(str.encode(ardY))
  
  #display the frame
  cv.imshow('frame', frame) 
  cv.imshow('green',green)
  cv.setMouseCallback('frame', onMouse) #coordinates of mouse click

  # the 'q' button is set as the 
  if cv.waitKey(1) & 0xFF == ord('q'): 
    break
  


vid.release()   # After the loop release the cap object 
cv.destroyAllWindows() # Destroy all the windows 