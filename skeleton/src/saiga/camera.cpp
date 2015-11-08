#include "camera.h"

#define ANG2RAD 3.14159265358979323846/180.0

Camera::Camera(const std::string &name) : name(name)
{
}


void Camera::setView(const mat4 &v){
    view=v;
    recalculateMatrices();
    model = glm::inverse(view);

    this->position = vec3(model[3]);
    //    recalculatePlanes();

}

void Camera::setView(const vec3 &eye,const vec3 &center,const vec3 &up){
    setView(glm::lookAt(eye,center,up));
}

void Camera::updateFromModel(){
    view = glm::inverse(model);
    recalculateMatrices();
}





std::ostream& operator<<(std::ostream& os, const Camera& ca){
    os<<ca.name;
    //    os<<"Nearplane= ("<<ca.nw*2<<" x "<<ca.nh*2<<") Farplane= ("<<ca.fw*2<<" x "<<ca.fh*2<<")";
    return os;
}



//===================================================================================================


void PerspectiveCamera::setProj(float fovy, float aspect, float zNear, float zFar){
    fovy = degreesToRadians(fovy);
    this->fovy = fovy;
    this->aspect = aspect;
    this->zNear = zNear;
    this->zFar = zFar;


    tang = (float)tan(fovy * 0.5) ;
    nh = zNear * tang;
    nw = nh * aspect;
    fh = zFar  * tang;
    fw = fh * aspect;

    proj = glm::perspective(fovy,aspect,zNear,zFar);
}

void PerspectiveCamera::recalculatePlanes()
{


}

std::ostream& operator<<(std::ostream& os, const PerspectiveCamera& ca){
    os<<"Type: Perspective Camera\n";
    os<<"Name='"<<ca.name<<"' Fovy="<<ca.fovy<<" Aspect="<<ca.aspect<<" zNear="<<ca.zNear<<" zFar="<<ca.zFar<<"\n";
    os<<static_cast<const Camera&>(ca);
    return os;
}

//=========================================================================================================================

void OrthographicCamera::setProj( float left, float right,float bottom,float top,float near,  float far){
    this->left = left;
    this->right = right;
    this->bottom = bottom;
    this->top = top;
    this->zNear = near;
    this->zFar = far;

    nh = (top-bottom)/2;
    nw = (right-left)/2;

    fh = nh;
    fw = nw;
//    fh = (top-bottom)/2;
//    fw = (right-left)/2;
    proj = glm::ortho(left,right,bottom,top,near,far);
}

void OrthographicCamera::recalculatePlanes()
{

}



std::ostream& operator<<(std::ostream& os, const OrthographicCamera& ca){
    os<<"Type: Orthographic Camera";
    os<<"Name='"<<ca.name<<"' left="<<ca.left<<" right="<<ca.right<<" bottom="<<ca.bottom<<" top="<<ca.top<<" zNear="<<ca.zNear<<" zFar="<<ca.zFar<<"\n";
    os<<static_cast<const Camera&>(ca);
    return os;
}

