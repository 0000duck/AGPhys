#include "saiga/opengl/shader/basic_shaders.h"
void MVPColorShader::checkUniforms(){
    MVPShader::checkUniforms();
    location_color = getUniformLocation("color");
}

void MVPColorShader::uploadColor(const vec4 &color){
    upload(location_color,color);
}



void MVPShader::checkUniforms(){
    Shader::checkUniforms();
    location_model = getUniformLocation("model");
    location_view = getUniformLocation("view");
    location_proj = getUniformLocation("proj");
    location_mv = getUniformLocation("MV");
    location_mvp = getUniformLocation("MVP");
}

void MVPShader::uploadAll(const mat4& m1,const mat4& m2,const mat4& m3){
    uploadModel(m1);
    uploadView(m2);
    uploadProj(m3);

    uploadMV(m2*m1);
    uploadMVP(m3*m2*m1);
}

