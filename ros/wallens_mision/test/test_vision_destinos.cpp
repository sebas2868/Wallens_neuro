#include "../src/vision_destinos.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <iostream>
int main(int argc,char** argv){
  for(int i=0;i<3;++i){
    std::string tipo=std::array<std::string,3>{"limon","naranja","mandarina"}[i];
    auto imagen=cv::imread(std::string(argv[1])+"/caja_"+tipo+".png");
    assert(!imagen.empty());
    auto marcos=vision_destinos::marcos(imagen);
    assert(marcos.size()==1 && marcos[0].tipo==tipo);
    vision_destinos::PoseCaja pose;
    assert(!vision_destinos::pose_caja(marcos[0],0,770,320,240,0.1,pose));
  }
  cv::Mat piso(480,640,CV_8UC3,cv::Scalar(70,70,70));
  cv::circle(piso,{320,320},25,cv::Scalar(0,255,255),-1);
  assert(vision_destinos::marcos(piso).empty());
  double rumbo;
  assert(!vision_destinos::sobre_baldosa(piso,320,320,1,770,320,256.6,63.4,rumbo));
  // Cuadrado sintético proyectado a 2.5 cm; fruta al centro a un metro.
  cv::Mat baldosa(480,640,CV_8UC3,cv::Scalar(30,30,30));
  std::vector<cv::Point> quad;
  for(auto p:std::vector<cv::Point2d>{{0.875,-0.125},{0.875,0.125},{1.125,0.125},{1.125,-0.125}})
    quad.emplace_back(cvRound(320-770*p.y/p.x),cvRound(256.6+(63.4-770*0.025)/p.x));
  cv::fillConvexPoly(baldosa,quad,cv::Scalar(220,220,220));
  assert(vision_destinos::sobre_baldosa(baldosa,320,256.6+63.4-770*0.025,1,770,320,256.6,63.4,rumbo));
  assert(!vision_destinos::sobre_baldosa(baldosa,320,256.6+63.4,1,770,320,256.6,63.4,rumbo));
  std::cout<<"Visión C++: 3 tarjetas reales correctas; ancho sin medir rechazado; fruta del piso rechazada; baldosa sintética aceptada.\n";
}
