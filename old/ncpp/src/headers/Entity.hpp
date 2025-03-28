
#ifndef KUROME_ENTITY_CLASS

#define KUROME_ENTITY_CLASS
#include "kmsgs.h"

extern int KUROME_ENTITY_ID_NUM;

/*! \class Entity
 * This class is built as basically a hitbox. It has functionallity for a lot of things.
 * We probably won't see it outside of being used as a hitbox though.
 */
class Entity {
public:
   double posx; /**< x position of the Entity */
   double posy; /**< y position of the Entity */
   double xwid; /**< width of the Entity */
   double ywid; /**< height of the Entity */
   double rot;  /**< current Entity rotation */
   int    type; /**< Entity type: Rectangle, Circle, or Point */
   int    val;  /**< Weight of the Entity */
   int    id;   /**< Unique id to associate the Entity with */
   
   Entity(double posx, double posy, double xwid, double ywid, int type, int val) 
      : posx(posx), posy(posy), xwid(xwid), ywid(ywid), rot(0.0), type(type), val(val) {
         KUROME_ENTITY_ID_NUM++;
         id = KUROME_ENTITY_ID_NUM;
      }

   Entity(double posx, double posy, double xwid, double ywid, double rot, int type, int val) 
      : posx(posx), posy(posy), xwid(xwid), ywid(ywid), rot(rot), type(type), val(val) {
         KUROME_ENTITY_ID_NUM++;
         id = KUROME_ENTITY_ID_NUM;
      }

   Entity(double xwid, double ywid, int type) 
      : posx(0), posy(0), xwid(xwid), ywid(ywid), rot(0.0), type(type), val(0) {
         KUROME_ENTITY_ID_NUM++;
         id = KUROME_ENTITY_ID_NUM;
      }

   Entity(struct entity_struct * es)
      : posx(es->posx), posy(es->posy), xwid(es->xwid), ywid(es->ywid), 
        rot(es->rot), type(es->type), val(es->val), id(es->id) {}

   /**
    * Populate the given struct with the
    * data from this entity.
    */
   void toStruct(struct entity_struct * es) {
      es->posx = posx;
      es->posy = posy;
      es->xwid = xwid;
      es->ywid = ywid;
      es->rot  = rot;
      es->type = type;
      es->val  = val;
      es->id   = id;
   }

   /**
    * Compute the distance squared between this
    * Entity and another Entity.
    */
   double dist2(const Entity & other) {
      return (((other.posx-posx)*(other.posx-posx))+((other.posy-posy)*(other.posy-posy)));
   }

   /**
    * Copy the data from another
    * Entity into this one.
    */
   void operator=(const Entity & other) {
      posx = other.posx;
      posy = other.posy;
      xwid = other.xwid;
      ywid = other.ywid;
      rot  = other.rot;
      type = other.type;
      val  = other.val;
      id   = other.id;
   }
};

#endif
