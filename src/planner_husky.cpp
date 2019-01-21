#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <time.h>
#include <limits.h>
#include <unistd.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <std_msgs/Char.h>
#include <cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

// std_msgs::Char cmd;
// ros::NodeHandle n;

char data;

//Bot colour is HEAD-blue TAIL-yellow

int white_thresh=80;//if BGR above this then treated as obstacle
int black_thresh=10;//if BGR below this then treated as black way
int blue_thresh=200; //if B value above than this and other values below side_thresh its blue
int yellow_thresh=220;//if G and R values greater than this and B value lower than side_thresh then yellow
int side_thresh=200;
float theta_thresh=10; // theta threshold between centroid of the bot and the waypoint/endpoint to consider as reached 
int circle_thresh=5;//min distance between centroid of the bot and the waypoint/endpoint to consider as reached 

Mat img;
ros::Publisher pub;
geometry_msgs::Twist cmd_vel;


typedef struct _Record
{
    Point prnt;
    double gdist;
    double fdist;
}Record;

typedef struct _pqRecord
{
    Point curr;
    double gdist;
    double fdist;
}pqRecord;  

typedef struct _blobData{
    int size = -1;
    Point centroid;
    bool color;   //true=blue  //false=yellow
}blobData;

typedef struct _Localization{
    Point centroid;
    float theta;
}Localisation;

void DrawRotatedRectangle(Mat& image, Point centerPoint,Size rectangleSize, double rotationDegrees)
{
    Scalar color = Scalar(255);           //color
    RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);
    Point vertices[4];    
    for(int i = 0; i < 4; ++i)
    {
        vertices[i] = vertices2f[i];
    }
    fillConvexPoly(image,vertices,4,color);
}

void DrawRotatedRectangle1(Mat& image, Point centerPoint,Size rectangleSize, double rotationDegrees)
{
    Scalar color = Scalar(0);           //color
    RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);
    Point vertices[4];    
    for(int i = 0; i < 4; ++i)
    {
        vertices[i] = vertices2f[i];
    }
    fillConvexPoly(image,vertices,4,color);
}

class Compare
{   
public:
    bool operator()(pqRecord a, pqRecord b)
    {
        return ( a.fdist > b.fdist );
    }
};

class PathPlanner //For Path Planning
{

    private:
    Mat orig;
    Point end;
    Mat obs_map;
    Point start;

    float distance( int k, int m , int i , int j  )
    {
        return sqrt( pow(k-i,2) + pow(m-j,2) );
    }
    
    float Heuristic( int k, int m , int i , int j )
    {
        return sqrt(pow(k-i,2)+pow(m-j,2));
        // return abs(k-i) + abs(m-j);
    }
    

    public:
    
    PathPlanner(Point S, Point F, Mat img ,Mat img1  )
    {
      start=S;
      end=F;
      obs_map=img.clone();
      orig=img1.clone();
    }


    Localisation getPath()
    {
        bool path_found=false;
        priority_queue <pqRecord, vector<pqRecord>, Compare> pq;
        bool visited[obs_map.rows][obs_map.cols]={ false } ;
        Record openlist[obs_map.rows][obs_map.cols];

        for (int i = 0; i < obs_map.rows ; ++i)
            for(int j = 0; j < obs_map.cols ; j++)
            {
                openlist[i][j].fdist=INT_MAX;
            }

        pqRecord node;
        node.curr=start;
        node.gdist=0;
        node.fdist=0;
        pq.push(node);

        visited[node.curr.x][node.curr.y]=true;     

        while( !pq.empty() )
        {
            pqRecord prst=pq.top();  
            pq.pop();

            if( prst.curr==end ) 
            {
                cout<<"Path Found"<<endl;
                path_found=true;
                break;
            } 

            for(int i=-1; i<2 ; i++ )
                for(int j=-1 ; j<2 ; j++ )
                {
                    if( prst.curr.x+i<0||prst.curr.y+j<0 || prst.curr.x+i>=obs_map.rows || prst.curr.y+j>=obs_map.cols ) continue;
                    if( obs_map.at<uchar>(prst.curr.x+i,prst.curr.y+j)==255 ) continue;


                    if( openlist[prst.curr.x+i][prst.curr.y+j].fdist > prst.gdist + Heuristic(end.x,end.y,prst.curr.x+i,prst.curr.y+j) + distance(0,0,i,j) )
                    {
                        openlist[prst.curr.x+i][prst.curr.y+j].fdist = prst.gdist + Heuristic(end.x,end.y,prst.curr.x+i,prst.curr.y+j) + distance(0,0,i,j);
                        openlist[prst.curr.x+i][prst.curr.y+j].gdist = prst.gdist + distance(0,0,i,j);
                        openlist[prst.curr.x+i][prst.curr.y+j].prnt  = prst.curr;
                    }
                    
                     
                    if( visited[prst.curr.x+i][prst.curr.y+j] == false )
                    {
                        visited[prst.curr.x+i][prst.curr.y+j] = true;

                        pqRecord newNode;
                        newNode.curr.x=prst.curr.x+i;
                        newNode.curr.y=prst.curr.y+j;
                        newNode.gdist = prst.gdist + Heuristic(0,0,i,j);
                        newNode.fdist = prst.gdist + Heuristic(end.x,end.y,prst.curr.x+i,prst.curr.x+j) + distance(0,0,i,j);
                        pq.push(newNode);

                    }
                   
                     
                }                                                    
        } 
        Mat check( obs_map.size(),CV_8UC3,Scalar(0,0,0));
        for (int i = 0; i < obs_map.rows; ++i)
        {
            for (int j = 0; j < obs_map.cols; ++j)
            {
                if( obs_map.at<uchar>(i,j)==255 ) 
                {
                        check.at<Vec3b>(i,j)[0]=255;
                        check.at<Vec3b>(i,j)[1]=255;
                        check.at<Vec3b>(i,j)[2]=255;
                }
            }
        }

        Mat temp = orig.clone();
        Point C,D;
        bool cornerDetected;
        float thetaCurr=0,thetaPrev=0;

        Localisation wptCurr,wptPrev,wptCorner;
        int counter=10 , corner= 0;

        C.x=end.x;
        C.y=end.y;

        wptPrev.centroid.x=C.x;
        wptPrev.centroid.y=C.y;
        wptPrev.theta=0;

        if( path_found )
        {
            while(!(C.x==start.x && C.y==start.y ))
            {
                D = openlist[C.x][C.y].prnt;
                thetaCurr=180*atan2((C.y-D.y),(C.x-D.x))/CV_PI;
                if( counter==10 )
                {
                    wptCurr=wptPrev;
                    wptPrev.centroid.x=D.x;
                    wptPrev.centroid.y=D.y;
                    wptPrev.theta=thetaCurr;
                    if( C.x==end.x && C.y==end.y)  wptCurr = wptPrev;
                    wptCorner=wptPrev;
                    counter=0;
                }               

                thetaPrev=thetaCurr;
                counter++;

                C=D;
            }
        }
        else 
        {
            cout<<"Path Not Found"<<endl;
            wptCurr.centroid=start;
            wptCurr.theta=0;
        }

        temp.at<Vec3b>(wptCurr.centroid.x,wptCurr.centroid.y)[0]=0;
        temp.at<Vec3b>(wptCurr.centroid.x,wptCurr.centroid.y)[1]=255;
        temp.at<Vec3b>(wptCurr.centroid.x,wptCurr.centroid.y)[2]=0;
        namedWindow("Path",1);
        imshow("Path",temp);
        waitKey(10);     
        return wptCurr;
    }

};

Localisation BFS_VISIT(Mat img ) //For Localisation
{
    int s,t;
    int count=2;
    bool visited[img.rows][img.cols] = { false };

    queue<Point> P;
    vector<blobData> blobs;

    for( int i=0;i<img.rows;i++)
        for( int j=0;j<img.cols;j++)
        {
            if( visited[i][j]==false )
            if( (img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==0 )|| (img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==255 ))    
            {
                count++;

                visited[i][j]=true;

                blobData blob;
                blob.size=1;
                blob.centroid.x=i;
                blob.centroid.y=j;

                if( img.at<Vec3b>(i,j)[0]==255 ) blob.color=true;
                else blob.color=false;

                Point temp;
                temp.x=j;
                temp.y=i;
                P.push(temp);   
              
                while(!(P.empty()))
                {
                    Point curr=P.front();
                    P.pop();
                    s=curr.y;
                    t=curr.x;
                    for( int k=s-1; k<s+2 ; k++)
                        for( int m=t-1 ; m<t+2 ; m++)
                        {
                            if( k<0||m<0||k>=img.rows||m>=img.cols || visited[k][m] ) continue;
                            if( img.at<Vec3b>(k,m)[0]==255 && img.at<Vec3b>(k,m)[1]==0 && img.at<Vec3b>(k,m)[2]==0 && blob.color==true )    
                            {
                                visited[k][m]=true;

                                blob.size++;
                                blob.centroid.x+=k;
                                blob.centroid.y+=m;

                                Point temp;
                                temp.x=m;
                                temp.y=k;
                                P.push(temp);
                            }
                            if( (img.at<Vec3b>(k,m)[0]==0 && img.at<Vec3b>(k,m)[1]==255 && img.at<Vec3b>(k,m)[2]==255 ) && blob.color==false )  
                            {
                                visited[k][m]=true;

                                blob.size++;
                                blob.centroid.x+=k;
                                blob.centroid.y+=m;

                                Point temp;
                                temp.x=m;
                                temp.y=k;
                                P.push(temp);
                            }
                        }
        
                }
                blob.centroid.x/=blob.size; 
                blob.centroid.y/=blob.size; 
                blobs.push_back(blob);
            }
        }

        blobData *B=new blobData[2];
        for (int i = 0; i < blobs.size(); i++ )
        {
            if( blobs[i].color==true )
            {
                if( B[0].size<=blobs[i].size )
                {
                    B[0]=blobs[i];
                }
            }
            else
            {
                if( B[1].size<=blobs[i].size )
                {
                    B[1]=blobs[i];
                }
            }

        }
        Localisation S;

        S.centroid.x=(B[0].centroid.x + B[1].centroid.x)/2;
        S.centroid.y=(B[0].centroid.y + B[1].centroid.y)/2;
        
        double theta;
        theta=atan2(B[0].centroid.y - B[1].centroid.y,B[0].centroid.x - B[1].centroid.x) ;
        theta=theta*(180/3.1415);
        if (theta<0) theta += 360;
        S.theta=theta;
        
        return S;  
}

void Thresholding( Mat img , Mat imgc )
{
    // medianBlur(img, img, 3);
    for (int i = 0; i < img.rows; ++i)
    {
        for( int j=0;j< img.cols;j++)
        {
            //WHITE
            if( img.at<Vec3b>(i,j)[0]>white_thresh && img.at<Vec3b>(i,j)[1]>white_thresh && img.at<Vec3b>(i,j)[2]>white_thresh)
            {
                imgc.at<Vec3b>(i,j)[0]=255;
                imgc.at<Vec3b>(i,j)[1]=255;
                imgc.at<Vec3b>(i,j)[2]=255;
            }
            //BLACK
            if( img.at<Vec3b>(i,j)[0]<black_thresh && img.at<Vec3b>(i,j)[1]<black_thresh && img.at<Vec3b>(i,j)[2]<black_thresh)
            {
                imgc.at<Vec3b>(i,j)[0]=0;
                imgc.at<Vec3b>(i,j)[1]=0;
                imgc.at<Vec3b>(i,j)[2]=0;
            }
            //BLUE
            if( img.at<Vec3b>(i,j)[0]>180 && img.at<Vec3b>(i,j)[1]< 170 && img.at<Vec3b>(i,j)[2]<130)
            {
                imgc.at<Vec3b>(i,j)[0]=255;
                imgc.at<Vec3b>(i,j)[1]=0;
                imgc.at<Vec3b>(i,j)[2]=0;
            }
            //YELLOW
            if( img.at<Vec3b>(i,j)[0]<220 && img.at<Vec3b>(i,j)[1]>230 && img.at<Vec3b>(i,j)[2]>230)
            {
                imgc.at<Vec3b>(i,j)[0]=0;
                imgc.at<Vec3b>(i,j)[1]=255;
                imgc.at<Vec3b>(i,j)[2]=255;
            }
        }
    }
}

void publish( char c )
{
    if(c=='W') 
    {
        cmd_vel.linear.x=1.0;
        for(int i=0;i<250000;i++) pub.publish(cmd_vel);
        cmd_vel.linear.x=0.0;
        pub.publish(cmd_vel);
    }
    if(c=='A') 
    {  
        cmd_vel.angular.z=-1;
        for(int i=0;i<50000;i++) pub.publish(cmd_vel);
        cmd_vel.angular.z=0;
        pub.publish(cmd_vel);
    }
    if(c=='D') 
    {
        cmd_vel.angular.z= 1;
        for(int i=0;i<50000;i++) pub.publish(cmd_vel);
        cmd_vel.angular.z=0;
        pub.publish(cmd_vel);
    }
}

Localisation wptDetection( Mat img ,Point &c)
{/*
        for (int i = 0; i < img.rows; ++i)
            for (int j = 0; j < img.cols; ++j)  
            {
                if( i<= 6 || i >=479 || j<=38 || j>=539 ) 
                {
                    img.at<Vec3b>(i,j)[0]=0;
                    img.at<Vec3b>(i,j)[1]=0;
                    img.at<Vec3b>(i,j)[2]=0;
                }

            }*/
        Mat imgc=img.clone();
        Mat img1(img.size(),CV_8UC1,Scalar(0));
        
        Point S,F;
        int sx1=img.cols-1,sx2=0,sy1=img.rows-1,sy2=0;
        int ex1=img.cols-1,ex2=0,ey1=img.rows-1,ey2=0;
        
        Thresholding( img,imgc);
        
        imshow("Image",img);
        imshow("Thresh",imgc);

        Localisation B=BFS_VISIT(imgc);
        S.x=B.centroid.x ;
        S.y=B.centroid.y ;   
        double theta=B.theta;
        cout <<"Position : "<<B.centroid<<" Theta="<< theta<< endl;

        
        // DrawRotated Function takes S in inverted form thats why have to invert it 
        Point R,T;
        R.x=S.y;
        R.y=S.x;
        DrawRotatedRectangle1(imgc, R ,Size(10,10),360-theta);   // Changes            

        for(int i=0;i<img.rows;i++)
            for(int j=0;j<img.cols;j++)
            {
                if( imgc.at<Vec3b>(i,j)[0] >=220 && imgc.at<Vec3b>(i,j)[1]>=220 && imgc.at<Vec3b>(i,j)[2]>=220 )
                {
                    Point P;
                    P.x=j;
                    P.y=i;
                    DrawRotatedRectangle(img1,P ,Size(10,10),0);

                }
                
                if( imgc.at<Vec3b>(i,j)[0]<70 && imgc.at<Vec3b>(i,j)[1]<90 && imgc.at<Vec3b>(i,j)[2]>190 )
                {
                    if(ex1>i) ex1 = i;
                    if(ex2<i) ex2 = i;
                    if(ey1>j) ey1 = j;
                    if(ey2<j) ey2 = j;
                }
            }

        F.x=(ex1+ex2)/2;
        F.y=(ey1+ey2)/2;
        cout << "Start:" << S << "Finish:" << F << endl;
        
        c=S;

        T.x=F.y;
        T.y=F.x;
        DrawRotatedRectangle1(img1, T ,Size(10,10),0);           // Changes  
        DrawRotatedRectangle1(img1, R ,Size(10,10),360-theta);   // Changes            

        imshow("Img1",img1);

        PathPlanner Path(S,F,img1,img);
        Localisation wpt = Path.getPath();
        if( wpt.centroid == S ) 
        {
            DrawRotatedRectangle1(img1, R ,Size(20,20),360-theta);   // Changes
            PathPlanner Path_1(S,F,img1,img);
            wpt = Path_1.getPath();                         
        }
        return wpt;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cout<<"in"<<endl;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if( !img.data ) { printf("Error loading A \n"); return ; }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Pixel");
    ros::NodeHandle nh;
    char cmd;
    image_transport::ImageTransport it_(nh);
    image_transport::Subscriber image_sub_;
    image_sub_ = it_.subscribe("cam", 1,&imageCb);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    img=imread("/home/vib2810/catkin_ws/src/pixel/src/first.png");


    ros::Rate r(1);
    while(ros::ok())
    {
        // ros::spinOnce();
        // imshow("1",img);
        // ros::spinOnce();
        // imshow("2",img);
        // waitKey(1);
        ros::spinOnce();
        cout<<".";
        while( !img.data )
        {
            cout<<"No Image1"<<endl;
            continue;
        }

        // resize(img,img,Size(640,360));                 //change
        Point c;
        Localisation wpt=wptDetection(img,c);

        
        float theta_2 = atan2((wpt.centroid.y - c.y), (wpt.centroid.x - c.x))*180/CV_PI;
        if( theta_2<0 ) theta_2 += 360;
    
        // cap >>img;
        ros::spinOnce();

        while( !img.data )
        {
            cout<<"No Image2"<<endl;
            continue ;
        }

        Mat imgc = img.clone();
        Thresholding( img, imgc );
    
        Localisation L;
        L= BFS_VISIT( imgc );

        int count=0;
    
        float theta_1 = L.theta , theta_error = theta_2 - theta_1 ;
        if(theta_error>180) theta_error-=360;
        while (fabs(theta_error) > theta_thresh )              // thresholding
        {
            if( theta_error > 0 )
            {

                cmd='A';
                cout<<"A"<<endl;
                publish(cmd);

                // publishChar('A');
            }
            else 
            {
        
                cmd='D';
                cout<<"D"<<endl;
                publish(cmd);
                  

                
            }
            // waitKey(300);
            ros::spinOnce();

            // cap >> img;
            while( !img.data )
            {
                cout<<"No Image3"<<endl;
                continue ;
            }

            Thresholding( img, imgc );
            L= BFS_VISIT( imgc );
            theta_1 = L.theta ;
            theta_error = theta_2 - theta_1; 
            cout<<"Position : "<<L.centroid<<" Theta : "<<L.theta<<endl;
            if(theta_error>180) theta_error-=360;
            // if(theta_error>0) cout<< "DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD";
            // else cout<<"LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";

            count++;
            if( count > 3) break;
        }
        if (count <3)
        {

                Point curr_pos = L.centroid ;
        
                float curr_error,prev_error;
                curr_error=sqrt(pow(L.centroid.x - wpt.centroid.x,2)+ pow(L.centroid.y - wpt.centroid.y,2));
                prev_error=curr_error +10;
                
                count=0;
                while( curr_error > circle_thresh ) // thresholding
                {
                    curr_error=sqrt(pow(L.centroid.x - wpt.centroid.x,2)+ pow(L.centroid.y - wpt.centroid.y,2));
                    if( prev_error - curr_error > 0 )
                    {
                        cmd='W';
                       publish(cmd);
                        cout<<"W"<<endl;
        
                    }
                    else break;
                    // waitKey(300);
                    prev_error = curr_error;
        
                    // cap >> img;
                     ros::spinOnce();

        
                    while( !img.data )
                    {
                        cout<<"No Image4"<<endl;
                        continue ;
                    }
        
                    Thresholding( img, imgc );
                    L= BFS_VISIT( imgc );
                    // cout<<"tp"<<endl;
                    cout<<"Position : "<<L.centroid<<" Theta : "<<L.theta<<endl;
                    count++;
                    if( count>5 ) break;
                }
        }
        r.sleep();
    }
    return 0;
}

