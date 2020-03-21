"""
  Meaning of Localization:
  
  "Where is our car within a given map for an accuracy of 10 cm?"
   - GPS can not be trusted: Accurate within 10-50 metres.
   - Therefore we use Lidar and Radar Sensors to localize.
   - Use car's position (x,y) relative to world's global coordinates.
  

 What will we learn with localization:
  
  Bayes Filters
  Motion Models
  2-D Particle Filters 
  
 
 Probability Sense:
  
  Incoming: Red
  E.g Prior Distribution:
  | Green: 0.2 | Red: 0.2 | Red: 0.2 | Green: 0.2 | Green: 0.2 |
  
  Multiply : Correct by 0.6 and Incorrect by 0.2
  
  RESULT: | Green: 0.04 | Red: 0.12 | Red: 0.12 | Green: 0.04 | Green: 0.04 |
  Problem: Doesnt add up to 1, so divide by SUMMATION (which is 0.36)
  
  RESULT: | Green: 1/9 | Red: 1/3 | Red: 1/3 | Green: 1/9 | Green: 1/9 |
  
  Posterior Distribution: P(Xi|Z) Updated P(x) given after Red is selected

  Inexact Motion:
    What if we dont know where to go for sure we must 
    distribute to according properties.

    E.g | 0 | 0.5 | 0 | 0.5 | 0 |

    with P(Xi+2|Xi) = 0.8
         P(Xi+1|Xi) = 0.2
         P(Xi+3|Xi) = 0.2

    Result: | 0.4 | 0.005 | 0.005 | 0.4| 0.1 |

    E.g | 0.2 | 0.2 | 0.2 |  0.2 | 0.2 |

    with P(Xi+2|Xi) = 0.8
         P(Xi+1|Xi) = 0.2
         P(Xi+3|Xi) = 0.2
    
    Result: | 0.2 | 0.2 | 0.2 |  0.2 | 0.2 |

 Limit Distribution:
    What if the robots motion is going forever? 
    What will be probability at infinity steps?
    
    E.g: | 1 | 0 | 0 | 0 | 0 |

    Uniform Distribution: 
    | 0.2 | 0.2 | 0.2 | 0.2 | 0.2 |

"""
#Modify the move function to accommodate the added 
#probabilities of overshooting or undershooting 
#the intended destination.

p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def move(p, U):
    q = []
    U = U % len(p)
    for i in range(len(p)):
        # Go back three to see value 
        s = pExact*(p[(i-U)])
        s = s + pOvershoot*(p[(i-U+1)])
        s = s + pUndershoot*(p[(i-U-1)])
        q.append(s)
    return q

for i in range(1000):
    p = move(p,1)
    # | 0.2 | 0.2 | 0.2 | 0.2 | 0.2 |

print (move(p, 1))