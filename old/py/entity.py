
ENTITY_TYPE_RECT   = 0
ENTITY_TYPE_POINT  = 1
ENTITY_TYPE_CIRCLE = 2

class Entity:
    """Entity class"""
    shape = 0;
    danger = 0;
    dist1 = 0.0
    dist2 = 0.0
    posX = 0.0
    posY = 0.0

    def __init__(self, danger, shape, dist1, dist2, posX, posY):
        self.shape = shape
        self.danger = danger
        self.dist1 = dist1
        self.dist2 = dist2
        self.posX = posX
        self.posY = posY

class WeightedEntity(Entity):
    """Extension to the Entity class that includes weight"""
    weight = 0.0

    def __init__(self,danger,shape,dist1,dist2,posX,posY,weight):
        super().__init__(danger,shape,dist1,dist2,posX,posY)
        self.weight = weight

class MovingEntity(Entity):
    """Extension to the Entity class that has some vars for movement"""
    turnRad = 0.0
    angle = 0.0

    def __init__(self,danger,shape,dist1,dist2,posX,posY,turnRad,angle):
        super().__init__(danger,shape,dist1,dist2,posX,posY)
        self.turnRad = turnRad
        self.angle = angle



