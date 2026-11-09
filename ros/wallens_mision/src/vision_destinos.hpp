// Geometría de visión sin ROS: tarjetas y soporte claro de la baldosa. No abre cámaras.
#pragma once
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace vision_destinos {
struct Marco {std::string tipo; std::vector<cv::Point2f> esquinas;};
inline std::vector<cv::Point2f> ordenar(const std::vector<cv::Point>& p) {
  std::vector<cv::Point2f> q(4);
  auto suma=[](auto a, auto b){return a.x+a.y < b.x+b.y;};
  auto resta=[](auto a, auto b){return a.y-a.x < b.y-b.x;};
  q[0]=*std::min_element(p.begin(),p.end(),suma); q[2]=*std::max_element(p.begin(),p.end(),suma);
  q[1]=*std::min_element(p.begin(),p.end(),resta); q[3]=*std::max_element(p.begin(),p.end(),resta);
  return q;
}
inline std::vector<Marco> marcos_umbral(const cv::Mat& bgr, bool suavizar, double resta) {
  cv::Mat gris, negro;
  cv::cvtColor(bgr, gris, cv::COLOR_BGR2GRAY);
  const double escala=640.0/std::max(gris.rows, gris.cols);
  cv::resize(gris,gris,cv::Size(),escala,escala);
  if(suavizar)cv::GaussianBlur(gris, gris, cv::Size(3,3),0);
  cv::adaptiveThreshold(gris,negro,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY_INV,31,resta);
  std::vector<std::vector<cv::Point>> contornos;
  std::vector<cv::Vec4i> jer;
  cv::findContours(negro,contornos,jer,cv::RETR_TREE,cv::CHAIN_APPROX_NONE);
  std::vector<Marco> salida;
  for(size_t i=0;i<contornos.size();++i) {
    double area=cv::contourArea(contornos[i]);
    if(area<120 || jer[i][2]<0) continue;
    std::vector<cv::Point> pol;
    cv::approxPolyDP(contornos[i],pol,0.03*cv::arcLength(contornos[i],true),true);
    if(pol.size()!=4 || !cv::isContourConvex(pol)) continue;
    auto esq=ordenar(pol);
    std::array<double,4> lados;
    for(int j=0;j<4;++j) lados[j]=cv::norm(esq[j]-esq[(j+1)%4]);
    if(*std::min_element(lados.begin(),lados.end())<=0 ||
       *std::max_element(lados.begin(),lados.end())>1.6* *std::min_element(lados.begin(),lados.end())) continue;
    int hueco=jer[i][2];
    for(int h=hueco;h>=0;h=jer[h][0])
      if(cv::contourArea(contornos[h])>cv::contourArea(contornos[hueco])) hueco=h;
    if(cv::contourArea(contornos[hueco])<0.5*area) continue;
    std::vector<cv::Point2f> rincones={{0,0},{200,0},{200,200},{0,200}}, pts, enderezados;
    for(auto p:contornos[hueco]) pts.emplace_back(p);
    cv::perspectiveTransform(pts,enderezados,cv::getPerspectiveTransform(esq,rincones));
    std::array<double,4> dist;
    for(int j=0;j<4;++j){dist[j]=1e9;for(auto p:enderezados)dist[j]=std::min(dist[j],cv::norm(p-rincones[j]));}
    auto valores=dist;std::sort(valores.begin(),valores.end());
    int mordida=std::max_element(dist.begin(),dist.end())-dist.begin();
    if(valores[3]<=1.6*valores[1] || valores[3]-valores[2]<=6 || mordida==3) continue;
    for(auto& p:esq)p*=1.0/escala;
    salida.push_back({std::array<std::string,3>{"naranja","mandarina","limon"}[mordida],esq});
  }
  return salida;
}
inline std::vector<Marco> marcos(const cv::Mat& bgr) {
  auto salida=marcos_umbral(bgr,true,25);
  // C270: el marco fino se cortaba al suavizar y restar 25. Conservar las esquinas sin desenfoque.
  for(auto& nuevo:marcos_umbral(bgr,false,5)) {
    cv::Point2f centro;
    for(auto p:nuevo.esquinas)centro+=p*0.25f;
    bool existe=false;
    for(auto& viejo:salida) {
      cv::Point2f otro;
      for(auto p:viejo.esquinas)otro+=p*0.25f;
      if(cv::norm(otro-centro)<10)existe=true;
    }
    if(!existe)salida.push_back(nuevo);
  }
  return salida;
}
inline std::vector<cv::Rect> turquesas(const cv::Mat& bgr) {
  cv::Mat hsv,mask,labels,stats,centros;
  cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
  cv::inRange(hsv,cv::Scalar(84,60,70),cv::Scalar(104,255,255),mask);
  cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::Mat::ones(3,3,CV_8U));
  cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::Mat::ones(9,9,CV_8U));
  int n=cv::connectedComponentsWithStats(mask,labels,stats,centros);
  std::vector<cv::Rect> salida;
  for(int i=1;i<n;++i){
    cv::Rect r(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP),
               stats.at<int>(i,cv::CC_STAT_WIDTH),stats.at<int>(i,cv::CC_STAT_HEIGHT));
    if(stats.at<int>(i,cv::CC_STAT_AREA)>=1200 && r.width>=r.height)salida.push_back(r);
  }
  return salida;
}
struct PoseCaja {double x,y,rumbo;};
inline bool pose_caja(const Marco& m, double ancho, double f, double cx, double cy, double cam_x, PoseCaja& p) {
  if(!(ancho>0 && std::isfinite(ancho))) return false; // POR MEDIR: no inventar escala
  double h=ancho/2;
  std::vector<cv::Point3f> mundo={{float(-h),float(-h),0},{float(h),float(-h),0},{float(h),float(h),0},{float(-h),float(h),0}};
  cv::Mat k=(cv::Mat_<double>(3,3)<<f,0,cx,0,f,cy,0,0,1), rv,tv,r;
  if(!cv::solvePnP(mundo,m.esquinas,k,cv::Mat(),rv,tv,false,cv::SOLVEPNP_IPPE))return false;
  if(tv.at<double>(2)<=0)return false;
  cv::Rodrigues(rv,r);
  double nx=r.at<double>(0,2),nz=r.at<double>(2,2);
  if(nz>0){nx=-nx;nz=-nz;}
  double n=std::hypot(nx,nz); if(n<0.5)return false;
  // La tarjeta está en el fondo: desplazar 16.5 cm hacia el frente de la caja.
  p={cam_x+tv.at<double>(2)+0.165*nz/n,-tv.at<double>(0)-0.165*nx/n,std::atan2(-nx,nz)};
  return true;
}
inline bool sobre_baldosa(const cv::Mat& bgr, double u, double fondo, double z, double f, double cx,
                         double horizonte, double k_suelo, double& rumbo) {
  if(z<=0)return false;
  double altura=(k_suelo-z*(fondo-horizonte))/f;
  if(altura<0.015 || altura>0.035)return false; // 2.5 cm ±1 cm; además exige el cuadrado
  cv::Mat gris,claro;
  cv::cvtColor(bgr,gris,cv::COLOR_BGR2GRAY);
  cv::threshold(gris,claro,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
  std::vector<std::vector<cv::Point>> contornos;
  cv::findContours(claro,contornos,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  for(auto& c:contornos){
    // Cámara baja: el cuadrado es un trapecio muy delgado en la imagen; 2.5% lo colapsaba a dos puntos.
    std::vector<cv::Point> q;cv::approxPolyDP(c,q,0.005*cv::arcLength(c,true),true);
    if(q.size()!=4 || !cv::isContourConvex(q) || cv::pointPolygonTest(q,cv::Point2f(u,fondo),false)<0)continue;
    std::vector<cv::Point2d> piso;
    for(auto p:q){
      if(p.y<=horizonte+2){piso.clear();break;}
      double d=(k_suelo-f*0.025)/(p.y-horizonte);
      piso.emplace_back(d,-(p.x-cx)*d/f);
    }
    if(piso.size()!=4)continue;
    bool valida=true;
    for(int i=0;i<4;++i){double lado=cv::norm(piso[i]-piso[(i+1)%4]);if(lado<0.18||lado>0.32)valida=false;}
    if(valida){rumbo=std::atan2(piso[1].y-piso[0].y,piso[1].x-piso[0].x);return true;}
  }
  return false;
}
} // namespace vision_destinos
