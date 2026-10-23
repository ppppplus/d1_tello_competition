// Load our GUI configuration directly, without rewriting user Gazebo defaults.
#include <ignition/gazebo/gui/Gui.hh>
#include <QSurfaceFormat>
int main(int argc,char **argv){
  if(argc!=2)return 2;
  QSurfaceFormat format;
  format.setVersion(3,3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  QSurfaceFormat::setDefaultFormat(format);
  auto app=ignition::gazebo::gui::createGui(argc,argv,argv[1],argv[1],false,"ogre2");
  return app?app->exec():1;
}
