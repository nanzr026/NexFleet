"""Pure product validation used by the Webots environment adapter."""
import math
OFFSET=1.25
def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))
def pickup_rejection(robot,product,speed,carrier,state):
    if not all(math.isfinite(v) for v in (*robot,*product,speed)):return "INVALID_MEASUREMENT"
    if carrier is not None or state!="AVAILABLE":return "PRODUCT_UNAVAILABLE"
    x,y,yaw=robot
    if abs(math.dist((x,y),product[:2])-OFFSET)>.20:return "PICKUP_DISTANCE"
    if abs(wrap(math.atan2(product[1]-y,product[0]-x)-yaw))>.10:return "PICKUP_ALIGNMENT"
    if speed>.03:return "ROBOT_MOVING"
    return None
def drop_rejection(robot,speed,carrier,requester,delivery):
    if not all(math.isfinite(v) for v in (*robot,speed,*delivery)):return "INVALID_MEASUREMENT"
    if carrier!=requester:return "NOT_CARRIER"
    if speed>.03:return "ROBOT_MOVING"
    x,y,yaw=robot
    if math.dist((x+OFFSET*math.cos(yaw),y+OFFSET*math.sin(yaw)),delivery)>.23:return "DROP_LOCATION"
    return None
def released_at(product,delivery,carrier,state):
    return carrier is None and state=="DELIVERED" and math.dist(product[:2],delivery)<.23

def rectangles_overlap(a,b):
    """Separating-axis test for measured oriented rectangle footprints."""
    def corners(rect):
        x,y,hx,hy,yaw=rect;c,s=math.cos(yaw),math.sin(yaw)
        return [(x+u*c-v*s,y+u*s+v*c) for u,v in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
    ca,cb=corners(a),corners(b)
    for angle in (a[4],a[4]+math.pi/2,b[4],b[4]+math.pi/2):
        axis=(math.cos(angle),math.sin(angle))
        pa=[x*axis[0]+y*axis[1] for x,y in ca]
        pb=[x*axis[0]+y*axis[1] for x,y in cb]
        if max(pa)<=min(pb) or max(pb)<=min(pa):return False
    return True
