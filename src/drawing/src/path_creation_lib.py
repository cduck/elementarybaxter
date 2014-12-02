#!/usr/bin/env python

import sys, copy
import numpy as np
import math


fSize = 0.07

capScale = (2/(math.sqrt(5)+1), 1)
thinScale = (.7*capScale[0],capScale[1])
skinnyScale = (.2*capScale[0],capScale[1])
hSpace = .3
vHeight = capScale[1]+hSpace
zClearance = .04
seqOnce = 'once'
seqLoop = 'loop'
symbolLib = {
  'A': (((0,0),(.5,1),(1,0),(.75,.5),(.25,.5)),        seqOnce, capScale),
  'B': (((0,0),(0,1),(1,.9),(1,.6),(0,.5),(1,.4),(1,.1)), seqLoop, capScale),
  'C': (((1,.85),(.8,1),(.2,1),(0,.8),(0,.2),(.2,0),(.8,0),(1,.15)), seqOnce, capScale),
  'D': (((0,0),(0,1),(1,.7),(1,.3)),                   seqLoop, capScale),
  'E': (((1,1),(0,1),(0,0),(1,0),None,(0,.5),(.7,.5)), seqOnce, capScale),
  'F': (((1,1),(0,1),(0,0),(0,.5),(.7,.5)),            seqOnce, capScale),
  'G': (((1,.85),(.8,1),(.2,1),(0,.8),(0,.2),(.2,0),(.8,0),(1,.1),(1,.3),(.7,.3)), seqOnce, capScale),
  'H': (((0,1),(0,0),(0,.5),(1,.5),(1,1),(1,0)),       seqOnce, capScale),
  'I': (((0,0),(1,0),(.5,0),(.5,1),(0,1),(1,1)),       seqOnce, thinScale),
  'J': (((.2,1),(1,1),(.6,1),(.6,.0),(0,0),(0,.3)),    seqOnce, capScale),
  'K': (((0,1),(0,0),(0,.5),(1,0),(1,1)),        (0,1,2,3,2,4), capScale),
  'L': (((0,1),(0,0),(1,0)),                           seqOnce, capScale),
  'M': (((0,0),(0,1),(.5,.5),(1,1),(1,0)),          seqOnce, capScale),
  'N': (((0,0),(0,1),(1,0),(1,1)),                     seqOnce, capScale),
  'O': (((0,0),(1,0),(1,1),(0,1)),                     seqLoop, capScale),
  'P': (((0,0),(0,1),(1,1),(1,.5),(0,.5)),             seqOnce, capScale),
  'Q': (((1,0),(1,1),(0,1),(0,0),(.6,.4),(1.2,-.2)), (0,1,2,3,0,4,5), capScale),
  'R': (((0,0),(0,1),(1,1),(1,.6),(0,.5),(1,0)),      seqOnce, capScale),
  'S': (((0,.15),(.2,0),(.8,0),(1,.15),(1,.35),(.8,.5),(.2,.5),(0,.65),(0,.85),(.2,1),(.8,1),(1,.85)), seqOnce, capScale),
  'T': (((0,1),(1,1),(.5,1),(.5,0)),                   seqOnce, capScale),
  'U': (((0,1),(0,.3),(.2,0),(.8,0),(1,.3),(1,1)),     seqOnce, capScale),
  'V': (((0,1),(.5,0),(1,1)),                          seqOnce, capScale),
  'W': (((0,1),(.15,0),(.5,.5),(.85,0),(1,1)),         seqOnce, capScale),
  'X': (((0,0),(1,1),None,(0,1),(1,0)), seqOnce, capScale),
  'Y': (((0,1),(.5,.5),(1,1),(.5,0)),              (0,1,2,1,3), capScale),
  'Z': (((0,1),(1,1),(0,0),(1,0),None,(.2,.5),(.8,.5)), seqOnce, capScale),
  '0': (((0,0),(1,0),(1,1),(0,1)),               (0,1,2,3,0,2), capScale),
  '1': (((0,.7),(.5,1),(.5,0),(0,0),(1,0)),            seqOnce, thinScale),
  '2': (((0,.85),(.2,1),(.8,1),(1,.85),(1,.6),(0,0),(1,0)), seqOnce, capScale),
  '3': (((0,.85),(.2,1),(.8,1),(1,.85),(1,.65),(.8,.5),(.2,.5),(.8,.5),(1,.35),(1,.15),(.8,0),(.2,0),(0,.15)), seqOnce, capScale),
  '4': (((.4,1),(0,.5),(1,.5),None,(1,1),(1,0)), seqOnce, capScale),
  '5': (((0,.15),(.2,0),(.8,0),(1,.15),(1,.35),(.8,.5),(.2,.5),(0,.35),(0,1),(1,1)), seqOnce, capScale),
  '6': (((0,.5),(.8,.5),(1,.35),(1,.15),(.8,0),(.2,0),(0,.15),(0,.5),(.15,.85),(.7,1)), seqOnce, capScale),
  '7': (((.2,0),(1,1),(0,1)), seqOnce, capScale),
  '8': (((.5,.5),(1,.65),(1,.85),(.8,1),(.2,1),(0,.85),(0,.65),(1,.35),(1,.15),(.8,0),(.2,0),(0,.15),(0,.35)), seqLoop, capScale),
  '9': (((.3,0),(.85,.15),(1,.5),(1,.85),(.8,1),(.2,1),(0,.85),(0,.65),(.2,.5),(1,.5)), seqOnce, capScale),
  ' ': ((),                                            seqOnce, capScale),
  '.': (((.5,0),),                                     seqOnce, skinnyScale),
  ',': (((0,-0.2),(1,.2)),                             seqOnce, skinnyScale),
  '!': (((.5,0),None,(.5,.25),(.5,1)),                 seqOnce, skinnyScale),
  '?': (((0,.65),(0,.85),(.2,1),(.8,1),(1,.85),(1,.65),(.5,.4),(.5,.25),None,(.5,0)), seqOnce, capScale),
  '=': (((0,.4),(1,.4),None,(0,.6),(1,.6)),            seqOnce, capScale),
  '+': (((0,.5),(1,.5),None,(.5,.83),(.5,.17)),        seqOnce, capScale),
  '-': (((0,.5),(1,.5)),                               seqOnce, capScale),
  '*': (((0,.31),(1,.69),None,(1,.31),(0,.69),None,(.5,.83),(.5,.17)), seqOnce, capScale),
  '/': (((0,0),(1,1)),                                 seqOnce, thinScale),
  '\\\\': (((0,1),(1,0)),                              seqOnce, thinScale),
  '>': (((0,.2),(1,.5),(0,.8)),                        seqOnce, capScale),
  '<': (((1,.2),(0,.5),(1,.8)),                        seqOnce, capScale),
  '#': (((0,.25),(1,.25),None,(1,.75),(0,.75),None,(.25,0),(.25,1),None,(.75,1),(.75,0)), seqOnce, capScale),
  '$': (((0,.15),(.2,0),(.8,0),(1,.15),(1,.35),(.8,.5),(.2,.5),(0,.65),(0,.85),(.2,1),(.8,1),(1,.85),None,(.5,1.2),(.5,-.2)), seqOnce, capScale),
  '%': (((0,0),(1,1),None,(.25,.75),None,(.75,.25)),   seqOnce, thinScale),
  '^': (((0,.6),(.5,1),(1,.6)),                        seqOnce, capScale),
  '&': (((1,.4),(.5,0),(0,.4),(1,.75),(.5,1),(0,.7),(1,0)), seqOnce, capScale),
  '(': (((1,0),(0,.25),(0,.75),(1,1)),                 seqOnce, thinScale),
  ')': (((0,0),(1,.25),(1,.75),(1,1)),                 seqOnce, thinScale),
  '_': (((0,0),(1,0)),                                 seqOnce, capScale),
  '"': (((.2,.7),(.4,1),None,(.8,1),(.6,.7)),          seqOnce, thinScale),
  "'": (((.4,.7),(.6,1)),                              seqOnce, thinScale),
  '\\1': (((.3,.8),None,(.7,.8),None,(0,.4),(.1,.1),(.4,0),(.6,0),(.9,.1),(1,.4)), seqOnce, (1,1)),
  '\\2': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\3': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\4': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\5': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\6': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\7': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\8': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\9': (((0,1),(1,0)),                               seqOnce, thinScale),
  '\\0': (((0,1),(1,0)),                               seqOnce, thinScale),
};



def transformWaypoint(wp, trans):
  return trans.dot(np.array((wp[0],wp[1],wp[2],1)))[0:3]
def transformWaypoints(wps, trans):
  return np.array([transformWaypoint(wp, trans) for wp in wps])

def strToWaypoints(str, trans=None, clearance=zClearance, size=None):
  if clearance is None: clearance = zClearance
  if size is None: size = fSize

  wps = []
  x = 0
  y = -vHeight
  lastWasUp = True
  specialChar = False

  for c in str:
    if c == '\\' and not specialChar:
      specialChar = True
    else:
      if specialChar:
        c = '\\'+c
        specialChar = False
      if symbolLib.has_key(c): desc = symbolLib[c]
      elif symbolLib.has_key(c.capitalize()): desc = symbolLib[c.capitalize()]
      else:
        desc = None
        if '\n' == c or '\\n' == c:
          x = 0
          y -= vHeight
      if desc is None: continue

      points = desc[0]
      sequence = desc[1] if len(desc) >= 2 else seqOnce
      scale = desc[2] if len(desc) >= 3 else (1,1)
      if sequence is seqOnce:
        pointSequence = points
      elif sequence is seqLoop:
        pointSequence = points + (points[0],)
      else:
        pointSequence = [None if i is None else points[i] for i in sequence]

      if len(pointSequence) >= 1:
        pointSequenceScale = [None if p is None else ((p[0]*scale[0]+x)*fSize,(p[1]*scale[1]+y)*fSize) for p in pointSequence]

        wps.append(pointSequenceScale[0]+(clearance,))
        lastWasUp = False
        for p in pointSequenceScale:
          if p is None:
            if not lastWasUp:
              wps.append((wps[-1][0],wps[-1][1],clearance))
              lastWasUp = True
          else:
            if lastWasUp:
              wps.append(p+(clearance,))
            wps.append(p+(0,))
            lastWasUp = False
        if not lastWasUp:
          wps.append(pointSequenceScale[-1]+(clearance,))
        lastWasUp = True

      x += scale[0]+hSpace

  if trans is None: return np.array(wps)
  else: return transformWaypoints(wps, trans)



def main():
  pass


if __name__ == '__main__':
  main()
