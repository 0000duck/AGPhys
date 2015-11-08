#pragma once

#include <saiga/util/glm.h>
#include <saiga/rendering/object3d.h>

#include <string>
#include <iostream>

using std::string;

class SAIGA_GLOBAL Camera : public Object3D{
public:
    std::string name;

    //    mat4 model;
    mat4 view;
    mat4 proj;
    mat4 viewProj;

    float zNear,  zFar;
    float nw,nh,fw,fh; //dimensions of near and far plane

    vec3 vertices[8];

    Camera(const std::string &name);

    void setView(const mat4 &v);
    void setView(const vec3 &eye,const vec3 &center,const vec3 &up);


    void setProj(const mat4 &p){proj=p;recalculateMatrices();}

    void updateFromModel();
    void recalculateMatrices(){viewProj = proj * view;}
    virtual void recalculatePlanes() = 0;



private:
    friend std::ostream& operator<<(std::ostream& os, const Camera& ca);
};

//========================= PerspectiveCamera =========================

class SAIGA_GLOBAL PerspectiveCamera : public Camera{
public:
    double fovy,  aspect;
    float tang;
    PerspectiveCamera(const std::string &name):Camera(name){}
    void setProj(float fovy, float aspect, float zNear, float zFar);
    friend std::ostream& operator<<(std::ostream& os, const PerspectiveCamera& ca);

    virtual void recalculatePlanes();
};

//========================= OrthographicCamera =========================

class SAIGA_GLOBAL OrthographicCamera : public Camera{
public:
    float left,right,bottom,top;
    OrthographicCamera(const std::string &name):Camera(name){}
    void setProj( float left, float right,float bottom,float top,float near,  float far);

    friend std::ostream& operator<<(std::ostream& os, const OrthographicCamera& ca);


    virtual void recalculatePlanes() ;

};

