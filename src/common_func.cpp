#include <tdle/common_func.h>

//euclideanDistance function: 
//return the Euclidean distance between 2 points
float euclideanDistance(vector<float> x, vector<float> y) {
    float sum = 0.0;
    for (int i = 0; i < x.size(); i++) {
        sum += pow(x[i] - y[i], 2);
    }
    return sqrt(sum);
}


//sign function
float sign(float n)
{
  if (n<0.0){return -1.0;}
  else{return 1.0;}
}


//Nearest function: 
//find the nearest vertex from x_rand
std::vector<float> Nearest(std::vector<std::vector<float> > X,std::vector<float> x_rand)
{
  float min=euclideanDistance(X[0],x_rand);
  int indx=0;
  for (int c=1;c<X.size();c++)
  {
    if (euclideanDistance(X[c],x_rand)<min)
    {
      min=euclideanDistance(X[c],x_rand);
      indx=c;
    }
  }
  return X[indx];
}


//Steer function
//Return x_new, which is a new point on the line connecting x_nearst and x_rand
//distance from x_nearst to x_new is stride. 
std::vector<float> Steer(std::vector<float> x_nearest,std::vector<float> x_rand,float stride)
{
  std::vector<float> x_new;
  float norm=euclideanDistance(x_nearest,x_rand);
  x_new.push_back( x_nearest[0]+(stride/(norm+1e-4))*(x_rand[0]-x_nearest[0]) );
  x_new.push_back( x_nearest[1]+(stride/(norm+1e-4))*(x_rand[1]-x_nearest[1]) );
  return x_new;
}

//gridValue function:
//returns grid value at location Xp 
//map data:  100 occupied   -1 unknown    0 free
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp)
{

  float resolution=mapData.info.resolution;
  float Xstartx=mapData.info.origin.position.x;
  float Xstarty=mapData.info.origin.position.y;
  int out;

  float width=mapData.info.width;
  std::vector<signed char> Data=mapData.data;
  // ROS_WARN_THROTTLE(0.5,"Xp[0]: %f; Xp[1]: %f",Xp[0],Xp[1]);
  float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
  // ROS_WARN_THROTTLE(0.5,"indx: %f; mapsize:%d",indx,Data.size());
  out=Data[int(indx)];
  return out;
}

// ObstacleFree function
//return value: -1: unknown   0: obstacle    1: free
int ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub)
{
  float rez=float(mapsub.info.resolution)*.2;
  float stepz=int(ceil(euclideanDistance(xnew,xnear))/rez);
  std::vector<float> xi=xnear;
  int obs=0;//obstacle 
  int unk=0;//unknown
  geometry_msgs::Point p;
  for (int c=0;c<stepz;c++)
  {
    xi=Steer(xi,xnew,rez);
    if (gridValue(mapsub,xi) >= 50){ obs=1; }
    if (gridValue(mapsub,xi) ==-1){ unk=1;	break;}
  }
  int out=0;
  xnew=xi;
  if (unk==1){  out=-1;}
    
  if (obs==1){  out=0;}
      
  if (obs!=1 && unk!=1){   out=1;}
  
  return out;
}
int ObstacleFree(geometry_msgs::Point point1, geometry_msgs::Point point2, nav_msgs::OccupancyGrid mapsub)
{
  
}

//Initialize visualization_msgs::Marker
void visMarkerInit(visualization_msgs::Marker &marker, std::string ID, 
    int type, int action, float scale, float r, float g, float b, float a)
{
    marker.header.frame_id= ID;
    marker.type = type; // 8:points, 5:line_list, 6:cube_list
    marker.action = action;// 0: add, 2: delete
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

void visMarkerSet (visualization_msgs::Marker &marker, std::vector<float> &point)
{
    geometry_msgs::Point p;
    p.x=point[0];
    p.y=point[1];
    p.z=0.0;
    marker.points.push_back(p);
}