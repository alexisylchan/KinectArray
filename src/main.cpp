#include <stdio.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkType.h>
#include <vtkCamera.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkRendererCollection.h>

#include <XnCppWrapper.h>

#include "DeviceManager.h"
#include "SensorDevice.h"

//Tracking
//#include <QTimer>
#include "vtkDeviceInteractor.h"
#include "vtkInteractionDeviceManager.h"
#include "vtkInteractorStyleTrackballCamera.h"  
#include "vtkVRPNTrackerCustomSensor.h"
#include "vtkVRPNTrackerCustomSensorStyleCamera.h"
#include "vtkVRPNTracker.h"
#include "vtkVRPNTrackerStyleCamera.h"
#include "vtkInteractionDeviceManager.h"
#include <vtkMatrix4x4.h>
#include <vtkMath.h>

#include "vtkConeSource.h"
#include "vtkSphereSource.h"

#include "vtkCubeSource.h" 

#include "vtkPolyDataMapper.h"
#include "vtkActor.h" 
#include "vtkTransform.h"

#include "vtkProperty.h"

#include "vtkLight.h"
#include "vtkLightKit.h"

#include <vrpn_Analog.h> 

#include "vtkVRPNPhantom.h"
#include "vtkVRPNPhantomStyleCamera.h"
#include "vtkMatrix4x4.h"
#define S_WIDTH 640
#define S_HEIGHT 480
#define PIXEL_SCALE 2 
 
#define TIMER_LENGTH 1
#define MOCK_HEAD 0
#define COMP_SUTHERLAND 0
#define COMP_GAMMA9 1
#define COMPUTE_KINECT_BOUNDS 0
int comp = -1;

bool drawData = false;
bool drawKinect = false;
bool cullDepth = false;
bool useTNG = false;
bool usePhantom = false;
bool drawInStereo = false;
bool useTracker = false;

struct vtkRenderGroup
{
	vtkSmartPointer<vtkPolyData> polyData_;
	vtkSmartPointer<vtkPolyDataMapper> mapper_;
	vtkSmartPointer<vtkActor> actor_;
};

DeviceManager* devMan;
std::vector<CameraDataPacket> cameraDataVector;
std::vector<vtkRenderGroup> renderGroups;
vtkRenderWindow* renwin  ;
vtkRenderWindow* datawin  ;


// Headtracking 
  const char* trackerAddress;
  double trackerOrigin[3];
  int sensorIndex;
  int origSensorIndex;  
  //inputInteractor 
  vtkDeviceInteractor* inputInteractor;

  //Need to reset renderer to handle load state
  vtkVRPNTrackerCustomSensorStyleCamera*  kinectTrackerStyleCamera;
  vtkVRPNTrackerCustomSensorStyleCamera*  sciDataStyleCamera;
  vtkVRPNTrackerCustomSensor* tracker1; 

  //Phantom
  vtkVRPNPhantomStyleCamera* phantomStyleCameraS;
  vtkVRPNPhantomStyleCamera* phantomStyleCameraG;
	 
	bool first = true;
	
 
	vtkActor* ConeActorS;
	vtkConeSource* ConeS;

	
	vtkActor* ConeActorG;
	vtkConeSource* ConeG;


	//Replace with data set
	vtkActor* CubeActor;
	vtkCubeSource* CubeSource;
 
	vtkMatrix4x4* intrinsicMat;
	vtkMatrix4x4* extrinsicMat;
	vtkMatrix4x4* rotate180YOpenNI;

	vtkLight* MainLight;
	vtkLightKit* LightKit;

	
	FILE* coordsLog;
	vrpn_Analog_Remote* tng1;
	
	class tng_user_callback
	{
	public:
	  char tng_name[vrpn_MAX_TEXT_LEN];
	  vtkstd::vector<unsigned> tng_counts;
	  int channelIndex;
	  double initialValue;
	};

	tng_user_callback *TNGC1;
	void VRPN_CALLBACK handleTNG(void *userdata,
	const vrpn_ANALOGCB t)
	{
	  tng_user_callback *tData=static_cast<tng_user_callback *>(userdata); 

	  // TNG 3B values go from 0 to 252. To get an eye separation of 0.0065 to be in the middle (126), we use delta  == 0.0065/126 ==
		double value = t.channel[tData->channelIndex];  
		double delta = value; /* - tData->initialValue; */
		vtkCamera* camera = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
		camera->SetEyeOffset( (((camera->GetDistance()/camera->O2Screen)*0.065/2.0)/126.0)*delta);//Initial Camera Offset is 0, so no need to add to the initial camera offset
				
		camera = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
		camera->SetEyeOffset( (((camera->GetDistance()/camera->O2Screen)*0.065/2.0)/126.0)*delta);//Initial Camera Offset is 0, so no need to add to the initial camera offset
				

	}
	

/**
	Function Declaration
*/
void initializeLights();

void initializeTracker();
void initializeTNG();
void initializeEyeAngle(vtkCamera* camera);
void initializeDevices();

void updatePolyData();
void timerCallback();

  
void createConeS(bool deleteOldCone)
{
	
	double position[3] = {-0.000033, -0.065609, -0.087861};
	double quat[4] = { -0.205897 ,-0.050476, -0.227901 , 0.950326};
	    double  matrix[3][3];
		double orientNew[3] ;
		if (deleteOldCone)
		{
			ConeActorS->Delete();
			ConeS->Delete();
		}
		 
		vtkMatrix4x4* cameraLightTransformMatrix = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix(); 
		cameraLightTransformMatrix->MultiplyPoint(position,position); 
		// Update Object Orientation

		//Change transform quaternion to matrix
		vtkMath::QuaternionToMatrix3x3(quat, matrix); 
		vtkMatrix4x4* RotationMatrix = vtkMatrix4x4::New();
		RotationMatrix->SetElement(0,0, matrix[0][0]);
		RotationMatrix->SetElement(0,1, matrix[0][1]);
		RotationMatrix->SetElement(0,2, matrix[0][2]); 
		RotationMatrix->SetElement(0,3, 0.0); 
		
		RotationMatrix->SetElement(1,0, matrix[1][0]);
		RotationMatrix->SetElement(1,1, matrix[1][1]);
		RotationMatrix->SetElement(1,2, matrix[1][2]); 
		RotationMatrix->SetElement(1,3, 0.0); 
		
		RotationMatrix->SetElement(2,0, matrix[2][0]);
		RotationMatrix->SetElement(2,1, matrix[2][1]);
		RotationMatrix->SetElement(2,2, matrix[2][2]); 
		RotationMatrix->SetElement(2,3, 1.0); 

		//cameraLightTransformMatrix->Multiply4x4(cameraLightTransformMatrix,RotationMatrix,RotationMatrix); 
		vtkTransform::GetOrientation(orientNew,RotationMatrix); 
 
	// ConeSource
    ConeS = vtkConeSource::New(); 
	ConeS->SetRadius(0.05);
	ConeS->SetHeight(0.1);  
	ConeS->SetDirection(orientNew); 

	//ConeS Mapper
    vtkPolyDataMapper* ConeMapper = vtkPolyDataMapper::New();
	ConeMapper->SetInput(ConeS->GetOutput());
    
	ConeActorS = vtkActor::New();
    ConeActorS->SetMapper(ConeMapper); 
	ConeActorS->SetPosition(position); 
	ConeActorS->GetProperty()->SetColor(0.5,0.5,0);
	ConeActorS->UseBoundsOff();
	ConeActorS->Modified();
 	vtkRenderer* renderer1 = datawin->GetRenderers()->GetFirstRenderer();
	renderer1->AddActor(ConeActorS);

	ConeS->Delete(); 
	ConeMapper->Delete();


}




void createConeG(bool deleteOldCone)
{ /*

 origin
 -0.018600 -0.117373 -0.054196 -0.083124 -0.111811 -0.011647  0.990178
 nearz
 -0.017090  0.005544  0.003920 -0.038565 -0.169748 -0.312001  0.933999
 farz
 -0.048872  0.073774  0.125514 -0.097257 -0.139057 -0.300971  0.938414
 topy
 -0.029943  0.202331  0.001229 -0.082207 -0.131866 -0.270780  0.950016
 bottomy
 -0.038055 -0.085279  0.038467 -0.131727 -0.232748 -0.324240  0.907383
 leftx
 -0.208597  0.085714 -0.033738 -0.181448 -0.225020 -0.130362  0.948392
 rightx
  0.165684  0.067863  0.041129  0.002439 -0.201601 -0.061625  0.977524*/



	double position[3] = {-0.018600, -0.117373, -0.054196};//{0.002582 , -0.059024 , -0.096408 };
	double quat[4] = { -0.083124, -0.111811, -0.011647,  0.990178};
	/*double quat[4] = { -0.19  ,0.05 , 0.95  , 0.26};*/

	//	double position[3] = {0.001167, -0.002429, -0.078878};//{0.002582 , -0.059024 , -0.096408 };
	//double quat[4] = { 0.035031  ,-0.118583 , 0.930093  , 0.345887};
	    double  matrix[3][3];
		double orientNew[3] ;
		if (deleteOldCone)
		{
			ConeActorG->Delete();
			ConeG->Delete();
		}
		 
		vtkMatrix4x4* cameraLightTransformMatrix = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix(); 
		cameraLightTransformMatrix->MultiplyPoint(position,position); 
		// Update Object Orientation

		//Change transform quaternion to matrix
		vtkMath::QuaternionToMatrix3x3(quat, matrix); 
		vtkMatrix4x4* RotationMatrix = vtkMatrix4x4::New();
		RotationMatrix->SetElement(0,0, matrix[0][0]);
		RotationMatrix->SetElement(0,1, matrix[0][1]);
		RotationMatrix->SetElement(0,2, matrix[0][2]); 
		RotationMatrix->SetElement(0,3, 0.0); 
		
		RotationMatrix->SetElement(1,0, matrix[1][0]);
		RotationMatrix->SetElement(1,1, matrix[1][1]);
		RotationMatrix->SetElement(1,2, matrix[1][2]); 
		RotationMatrix->SetElement(1,3, 0.0); 
		
		RotationMatrix->SetElement(2,0, matrix[2][0]);
		RotationMatrix->SetElement(2,1, matrix[2][1]);
		RotationMatrix->SetElement(2,2, matrix[2][2]); 
		RotationMatrix->SetElement(2,3, 1.0); 

		//cameraLightTransformMatrix->Multiply4x4(cameraLightTransformMatrix,RotationMatrix,RotationMatrix); 
		vtkTransform::GetOrientation(orientNew,RotationMatrix); 
 
	// ConeSource
    ConeG = vtkConeSource::New(); 
	ConeG->SetRadius(0.05);
	ConeG->SetHeight(0.1);  
	ConeG->SetDirection(orientNew); 

	//ConeG Mapper
    vtkPolyDataMapper* ConeMapper = vtkPolyDataMapper::New();
	ConeMapper->SetInput(ConeG->GetOutput());
    
	ConeActorG = vtkActor::New();
    ConeActorG->SetMapper(ConeMapper); 
	ConeActorG->SetPosition(position); 
	ConeActorG->GetProperty()->SetColor(0,0.5,0.5);
	 
	ConeActorG->UseBoundsOff();
	ConeActorG->Modified();
 	vtkRenderer* renderer1 = datawin->GetRenderers()->GetFirstRenderer();
	renderer1->AddActor(ConeActorG);

	ConeG->Delete(); 
	ConeMapper->Delete();


}


void updatePolyData()
{ 
	if (drawData )
	{
		if (first)
		{
			 double position[3] = {0, 0, 0};  
			CubeSource = vtkCubeSource::New();  

			//ConeS Mapper
			vtkPolyDataMapper* CubeMapper = vtkPolyDataMapper::New();
			CubeMapper->SetInput(CubeSource->GetOutput());
		    
			CubeActor = vtkActor::New();
			CubeActor->SetMapper(CubeMapper); 
			CubeActor->SetPosition(position); 
			//CubeActor->GetProperty()->SetOpacity(0.5);
	  
			datawin->GetRenderers()->GetFirstRenderer()->AddActor(CubeActor); 
			datawin->GetRenderers()->GetFirstRenderer()->ResetCamera();
		}
	 
	}
	if (drawKinect)
	{
		if (first)
		{

		intrinsicMat = vtkMatrix4x4::New();
		intrinsicMat->SetElement(0,0,525.844);
		intrinsicMat->SetElement(0,1,0);
		intrinsicMat->SetElement(0,2,312.569); 
		intrinsicMat->SetElement(0,3,0); 

		intrinsicMat->SetElement(1,0,0);
		intrinsicMat->SetElement(1,1,527.421);
		intrinsicMat->SetElement(1,2,252.614); 
		intrinsicMat->SetElement(1,3,0); 

		intrinsicMat->SetElement(2,0,0);
		intrinsicMat->SetElement(2,1,0);
		intrinsicMat->SetElement(2,2,1); 
		intrinsicMat->SetElement(2,3,0); 
		extrinsicMat = vtkMatrix4x4::New();
		extrinsicMat->SetElement(0,0,-0.96094);
		extrinsicMat->SetElement(0,1,0.0781025);
		extrinsicMat->SetElement(0,2,0.265509); 
		extrinsicMat->SetElement(0,3,14.412); 

		extrinsicMat->SetElement(1,0,-0.0805999);
		extrinsicMat->SetElement(1,1,-0.996745);
		extrinsicMat->SetElement(1,2,0.00149397); 
		extrinsicMat->SetElement(1,3,16.1328); 

		extrinsicMat->SetElement(2,0,264762);
		extrinsicMat->SetElement(2,1,-0.0199644);
		extrinsicMat->SetElement(2,2,0.964107); 
		extrinsicMat->SetElement(2,3,52.6665); 

		 		
		coordsLog = fopen("coordslog-posttransformedbounds5.txt","w");
		}

	for(unsigned int i = 0; i < cameraDataVector.size(); i++)
	{
	
		// need to create a new render group for any new camera device
		if (renderGroups.size() <= i)
		{
			vtkRenderGroup renderGroup;
			renderGroup.polyData_ = vtkSmartPointer<vtkPolyData>::New();
			renderGroup.mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
			renderGroup.actor_ = vtkSmartPointer<vtkActor>::New();
			
			renderGroup.mapper_->SetInput(renderGroup.polyData_);
			renderGroup.actor_->SetMapper(renderGroup.mapper_);
			renwin->GetRenderers()->GetFirstRenderer()->AddActor(renderGroup.actor_);
			
			renderGroups.push_back(renderGroup);
		}
		double* cameraposition ;
		cameraposition = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetPosition( );
		
		vtkMatrix4x4* viewMatrix ;
		viewMatrix = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix();

		double* orientation;
		orientation = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetOrientation();

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkUnsignedCharArray> color = vtkSmartPointer<vtkUnsignedCharArray>::New();
		color->SetName("Color");
		color->SetNumberOfComponents(3);
	
		
		// These coordinates are in model view!!!!
		for(int y = 0; y < S_HEIGHT; y++)
		{
			for(int x = 0; x < S_WIDTH; x++)
			{ 
					XnDepthPixel d = cameraDataVector[i].depthMap_[x+y*S_WIDTH];
					XnRGB24Pixel c = cameraDataVector[i].imageMap_[x+y*S_WIDTH];
		
					vtkIdType id;

					double pixPos[4];
					XnFieldOfView fov;
					
					devMan->GetDeviceByIndex(0)->GetDepthGenerator()->GetFieldOfView(fov); 

					pixPos[0] = ((x - intrinsicMat->GetElement(0,2))*d)/intrinsicMat->GetElement(0,0) ; 
					pixPos[1] = ((y - intrinsicMat->GetElement(1,2))*d)/intrinsicMat->GetElement(1,1); 
					pixPos[2] = d; 
					pixPos[3] = 0;
					double transformedPixPos[4];
				 
					 extrinsicMat->MultiplyPoint(pixPos,transformedPixPos);  
					transformedPixPos[0] = pixPos[0]/(fov.fHFOV/2.0) ; 
					transformedPixPos[1] = pixPos[1]/(fov.fVFOV/2.0); 
					transformedPixPos[2] = COMPUTE_KINECT_BOUNDS ? d: d/(devMan->GetDeviceByIndex(0)->GetDepthGenerator()->GetDeviceMaxDepth()/2.0); 
					transformedPixPos[3] = 0;  
	
					if ( COMPUTE_KINECT_BOUNDS && first )
					{
						

						fprintf(coordsLog,"%f , %f , %f, %f\n",transformedPixPos[0],transformedPixPos[1],transformedPixPos[2],transformedPixPos[3]);
					}  

					 
					/*scale 
					xMax 2516.280273 , xMin -1051.145264 , 
					yMax 2277.582031 , yMin -416.293884, 
					zMax 0.000000, zMin -4406.604004, 
					wMax 0.000000, wMin 0.000000*/
						/*scale 2
					 xMax 2104.453125 , xMin -4754.977539 
					 yMax 2113.515137 , yMin -5464.675781, 
					 zMax 1.056400, zMin 0.000000, 
					 wMax 0.000000, wMin 0.000000*/

					transformedPixPos[0] = transformedPixPos[0]/((1507.638916   + 4345.924316  )/2.0);
					transformedPixPos[1] = transformedPixPos[1]/((1981.828125   + 3764.186279)/2.0);
					transformedPixPos[2] = transformedPixPos[2]/((4093.000000 + 0)/2.0);

					if ( /*d == 0  || */ (cullDepth && d > 1000))/*(d == 0)*/
					{
						id = points->InsertNextPoint(transformedPixPos[0],transformedPixPos[1],-1.0); 
						color->InsertNextTuple3(0,0,0);
					}
					else  
					{
				 		id = points->InsertNextPoint(transformedPixPos[0],transformedPixPos[1],transformedPixPos[2]); 
						color->InsertNextTuple3(c.nRed, c.nGreen, c.nBlue);
						
					}
		
					cellArray->InsertNextCell(1);
					cellArray->InsertCellPoint(id); 
			}
		}
		if (first)
		{
			fclose(coordsLog);
		}
		renderGroups[i].polyData_->SetPoints(points);
		renderGroups[i].polyData_->SetVerts(cellArray);
		renderGroups[i].polyData_->GetPointData()->SetScalars(color);
		renderGroups[i].polyData_->Modified();
		renderGroups[i].polyData_->Update();
	}
}

			first = false;
}

class UpdateData:public vtkCommand
{
	public:
		static UpdateData *New()
		{
			UpdateData* up = new UpdateData;
			return up;
		}
		virtual void Execute(vtkObject* caller, unsigned long eid, void* clientdata)
		{ 
			if (useTNG)
			{
				tng1->mainloop();
			}
		    if (useTracker)
			{
				inputInteractor->Update(); 
			}
			
		 	if (drawKinect)
			{
				devMan->GetCameraDataForAllDevices(cameraDataVector);
			}
			updatePolyData();   
			renwin->Render();
			datawin->Render();
		}
};

void timerCallback()
{
	devMan->GetCameraDataForAllDevices(cameraDataVector);
	updatePolyData(); 
	inputInteractor->Update(); 
}

void initializeLights()
{ 
  LightKit = vtkLightKit::New();
  LightKit->AddLightsToRenderer(datawin->GetRenderers()->GetFirstRenderer());
}
void initializeTracker()
{
	//Tracker Options
	useTracker = 1;

	/********************** CHANGE THE SENSOR INDEX WHEN CHANGING THE HOST ********************/
	if (comp == COMP_SUTHERLAND)
	{
		if (MOCK_HEAD)
			trackerAddress =  "tracker@localhost";//
		else
			trackerAddress =  "Tracker0@tracker1-cs.cs.unc.edu";
	}
	else if (comp == COMP_GAMMA9)
	{
		trackerAddress =  "tracker@gamma9.cs.unc.edu"; 
	}
	else
	{
		trackerAddress = "tracker@1";
	}
	/********************** CHANGE THE SENSOR INDEX WHEN CHANGING THE HOST ********************/

	trackerOrigin[0] =  -7.88;
	trackerOrigin[1] =  -5.19;
	trackerOrigin[2] =  -1.04;
		
	sensorIndex = 0; 
	origSensorIndex = 0;
 
	if (comp == COMP_SUTHERLAND)
	{
		initializeEyeAngle( renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera() );
		initializeEyeAngle( datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera() );
	}
	initializeDevices( ); 
}

void initializeTNG()
{
		//TNG  
		char tngAddress[] = "tng3name@localhost";
		tng1 = new vrpn_Analog_Remote(tngAddress);
		TNGC1 = new tng_user_callback;
		TNGC1->channelIndex = sensorIndex; //TODO: Should this be origSensorIndex? How often do we reinitialize device 
		TNGC1->initialValue = 0;
		strncpy(TNGC1->tng_name,tngAddress,sizeof(TNGC1->tng_name));
		tng1->register_change_handler(TNGC1,handleTNG);

}
void initializeEyeAngle(vtkCamera* camera)
{      
	
		camera->SetEyeAngle(0);
		camera->SetHeadTracked(true);//useTracker);

		//Code from vtkCaveSynchronizedRenderers::SetDisplayConfig
		double DisplayX[4], DisplayY[4],DisplayOrigin[4];

		 
			
			DisplayOrigin[0]= 8.174900;
		 	DisplayOrigin[1]= 5.435796; 
			DisplayOrigin[2]= 0.808637;//-value; 

			DisplayX[0]=   8.192622;//value; 
			DisplayX[1]=    4.967024;//-value; 
			DisplayX[2]=    0.803074;//-value; 

			DisplayY[0]= 8.197782;//value; 
			DisplayY[1]=    4.963459;//value; 
			DisplayY[2]=     1.094415;//-value;     

			//Todo: factorize this
			vtkMatrix4x4* trackerTransformM = vtkMatrix4x4::New();
			trackerTransformM->SetElement(0,0,0);
			trackerTransformM->SetElement(0,1,-1);
			trackerTransformM->SetElement(0,2,0);
			trackerTransformM->SetElement(0,3,-1*trackerOrigin[1]);
			trackerTransformM->SetElement(1,0,0);
			trackerTransformM->SetElement(1,1,0);
			trackerTransformM->SetElement(1,2,1);
			trackerTransformM->SetElement(1,3, 1*trackerOrigin[2]);
			trackerTransformM->SetElement(2,0,-1);
			trackerTransformM->SetElement(2,1,0);
			trackerTransformM->SetElement(2,2,0); 
			trackerTransformM->SetElement(2,3,-1*trackerOrigin[0]);
			trackerTransformM->SetElement(3,0, 0);
			trackerTransformM->SetElement(3,1, 0 );
			trackerTransformM->SetElement(3,2,0); 
			trackerTransformM->SetElement(3,3,1);
			DisplayOrigin[3] = 1;
			DisplayX[3] = 1;
			DisplayY[3] = 1;
			trackerTransformM->MultiplyPoint(DisplayOrigin,DisplayOrigin);
			trackerTransformM->MultiplyPoint(DisplayX,DisplayX);
			trackerTransformM->MultiplyPoint(DisplayY,DisplayY); 
		 

		double xBase[3],yBase[3],zBase[3];
		//Get Vectors of screen
		for (int i =0; i < 3; ++i)
		{
		xBase[i] = DisplayX[i]-DisplayOrigin[i];
		yBase[i] = DisplayY[i]-DisplayX[i];
		}
		vtkMath::Cross(xBase,yBase,zBase);
		//Code from vtkCaveSynchronizedRenderers::SetSurfaceRotation
		vtkMatrix4x4* SurfaceRot = vtkMatrix4x4::New();
		vtkMath::Normalize( xBase );
		vtkMath::Normalize( yBase );
		vtkMath::Normalize( zBase );

		SurfaceRot->SetElement( 0, 0, xBase[0] );
		SurfaceRot->SetElement( 0, 1, xBase[1] );
		SurfaceRot->SetElement( 0, 2, xBase[2] );

		SurfaceRot->SetElement( 1, 0, yBase[0] );
		SurfaceRot->SetElement( 1, 1, yBase[1] );
		SurfaceRot->SetElement( 1, 2, yBase[2] );

		SurfaceRot->SetElement( 2, 0, zBase[0]);
		SurfaceRot->SetElement( 2, 1, zBase[1]);
		SurfaceRot->SetElement( 2, 2, zBase[2]);
		SurfaceRot->MultiplyPoint( DisplayOrigin, DisplayOrigin );
		SurfaceRot->MultiplyPoint( DisplayX, DisplayX );
		SurfaceRot->MultiplyPoint( DisplayY, DisplayY );  
		// Set O2Screen, O2Right, O2Left, O2Bottom, O2Top
		double O2Screen = - DisplayOrigin[2];
		double O2Right  =   DisplayX[0];
		double O2Left   = - DisplayOrigin[0];
		double O2Top    =   DisplayY[1];
		double O2Bottom = - DisplayX[1]; 
		camera->SetConfigParams(O2Screen,O2Right,O2Left,O2Top,O2Bottom, 0.065  ,/*6.69*/(camera->GetDistance()/(2*O2Screen)),SurfaceRot);
		camera->Modified(); 
		SurfaceRot->Delete();
		 
} 
void initializeDevices()
{ 

	/////////////////////////INTERACTOR////////////////////////////
	// Initialize Device Interactor to manage all trackers
    inputInteractor = vtkDeviceInteractor::New();
		/////////////////////////CREATE  TRACKER////////////////////////////

		//Create connection to VRPN Tracker using vtkInteractionDevice.lib
		tracker1 = vtkVRPNTrackerCustomSensor::New();
		tracker1->SetDeviceName(trackerAddress); 
		tracker1->SetSensorIndex(sensorIndex);//TODO: Fix error handling  
		tracker1->SetTracker2WorldTranslation(trackerOrigin[0],trackerOrigin[1],trackerOrigin[2]);
		double t2w1[3][3] = { 0, -1,  0,
							  0,  0, 1, 
							 -1, 0,  0 }; 
		double t2wQuat1[4];
		vtkMath::Matrix3x3ToQuaternion(t2w1, t2wQuat1);
		tracker1->SetTracker2WorldRotation(t2wQuat1);
		

		int trackerInitResult = tracker1->Initialize();
		if (trackerInitResult == 0)
		{
			printf("Error initializing tracker ");			 
		}

		/////////////////////////CREATE  TRACKER STYLE////////////////////////////

		//Create device interactor style (defined in vtkInteractionDevice.lib) that determines how the device manipulates camera viewpoint
		kinectTrackerStyleCamera = vtkVRPNTrackerCustomSensorStyleCamera::New();
		kinectTrackerStyleCamera->SetTracker(tracker1);
		kinectTrackerStyleCamera->SetRenderer(renwin->GetRenderers()->GetFirstRenderer());
			//Create device interactor style (defined in vtkInteractionDevice.lib) that determines how the device manipulates camera viewpoint
		sciDataStyleCamera = vtkVRPNTrackerCustomSensorStyleCamera::New();
		sciDataStyleCamera->SetTracker(tracker1);
		sciDataStyleCamera->SetRenderer(datawin->GetRenderers()->GetFirstRenderer());

	
		/////////////////////////INTERACTOR////////////////////////////
		//Register Tracker to Device Interactor
		inputInteractor->AddInteractionDevice(tracker1);
		inputInteractor->AddDeviceInteractorStyle(kinectTrackerStyleCamera);  
		inputInteractor->AddDeviceInteractorStyle(sciDataStyleCamera);

		 
		if (usePhantom)
		{

		
		/////////////////////////FIRST PHANTOM//////////////////////////// 
		vtkVRPNPhantom* phantomS = vtkVRPNPhantom::New();
		phantomS->SetDeviceName("Phantom0@sutherland.cs.unc.edu");
		phantomS->SetPhantom2WorldTranslation(0.000264,0.065412,0.0);//TODO: FIX
		phantomS->SetNumberOfButtons(2);
		phantomS->SetSensorIndex(sensorIndex);

		phantomS->Initialize();
		createConeS(false);
 
		phantomStyleCameraS = vtkVRPNPhantomStyleCamera::New();   
		phantomStyleCameraS->SetPhantomType(PHANTOM_TYPE_OMNI);
		phantomStyleCameraS->SetActor(ConeActorS); 
		phantomStyleCameraS->SetConeSource(ConeS);
 
		phantomStyleCameraS->SetPhantom(phantomS);
		phantomStyleCameraS->SetRenderer(datawin->GetRenderers()->GetFirstRenderer());  
		//Register Phantom to Device Interactor 
		inputInteractor->AddInteractionDevice(phantomS);
		inputInteractor->AddDeviceInteractorStyle(phantomStyleCameraS); 

		
		/////////////////////////SECOND PHANTOM//////////////////////////// 
		vtkVRPNPhantom* phantomG = vtkVRPNPhantom::New();   
		phantomG->SetDeviceName("Phantom0@gamma9.cs.unc.edu");
		phantomG->SetPhantom2WorldTranslation(0.000264,0.065412,0.0);//TODO: FIX
		phantomG->SetNumberOfButtons(2);
		phantomG->SetSensorIndex(sensorIndex);

		phantomG->Initialize();
		createConeG(false); 
		/////////////////////////CREATE  PHANTOM STYLE////////////////////////////
		phantomStyleCameraG = vtkVRPNPhantomStyleCamera::New(); 
		phantomStyleCameraG->SetPhantomType(PHANTOM_TYPE_DESKTOP);
		/////////////////////////CONNECT TO SERVER CHANGE////////////////////////////
 

		phantomStyleCameraG->SetActor(ConeActorG); 
		phantomStyleCameraG->SetConeSource(ConeG);
 
		phantomStyleCameraG->SetPhantom(phantomG);
		phantomStyleCameraG->SetRenderer(datawin->GetRenderers()->GetFirstRenderer()); 

		
	    /////////////////////////INTERACTOR////////////////////////////
		//Register Phantom to Device Interactor 
		inputInteractor->AddInteractionDevice(phantomG);
		inputInteractor->AddDeviceInteractorStyle(phantomStyleCameraG); 

		}
}

int main(int argc, char** argv)
{ 
	while (--argc > 0 )
	{    
		if (!strcmp(argv[argc] , "--sutherland"))
		{
			
			if (comp != COMP_GAMMA9)
				comp = COMP_SUTHERLAND;
			else
			{
				printf ("Error trying to assign app to two computers!");
				exit(1);
			} 
		}
		else if (!strcmp(argv[argc] , "--gamma9"))
		{
			if (comp != COMP_SUTHERLAND)
				comp = COMP_GAMMA9;
			else
			{
				printf ("Error trying to assign app to two computers!");
				exit(1);
			}
		}
		else if (!strcmp(argv[argc] , "--stereo"))
		{
			drawInStereo = true;
		}
		else if (!strcmp(argv[argc] , "--phantom"))
		{
			usePhantom = true;
		}
		else if (!strcmp(argv[argc] , "--tng"))
		{
			useTNG = true;
		}
		else if (!strcmp(argv[argc] , "--cull-depth"))
		{
			cullDepth = true;
		}
		else if (!strcmp(argv[argc] , "--kinect"))
		{
			drawKinect = true;
		}
		else if (!strcmp(argv[argc] , "--data"))
		{
			drawData = true;
		}
		else
		{
			printf ("The following argument is invalid %s.\n", argv[argc]);
			exit(1);
		}

	} 
	if (drawInStereo && comp != COMP_SUTHERLAND)
	{
		drawInStereo = false;
	}
	if (useTNG && comp != COMP_SUTHERLAND)
	{
		useTNG = false;
	}
	
	if (comp == COMP_SUTHERLAND || usePhantom || useTNG)
	{
		useTracker = true;
	}
	if (drawKinect)
	{
		devMan = new DeviceManager();
		
		unsigned int i = devMan->GetNODevicesConnected();
		
		if (i < 1)
		{
			std::cout << "No devices found." << std::endl;
			return -1;
		}
		
		printf("Number of devices connected: %d.\n", i);
	}
	//VTK Pipeline
	vtkRenderer* ren = vtkRenderer::New();
	ren->GetActiveCamera()->ParallelProjectionOff();
	vtkRenderer* dataren = vtkRenderer::New();
	dataren->GetActiveCamera()->ParallelProjectionOff();
	renwin = vtkRenderWindow::New();
	datawin= vtkRenderWindow::New();
	vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
	vtkRenderWindowInteractor* data_iren = vtkRenderWindowInteractor::New();
 
	
	if (drawInStereo)
	{
		renwin->SetStereoCapableWindow(1);
		renwin->StereoRenderOn();
		
		datawin->SetStereoCapableWindow(1);
		datawin->StereoRenderOn();
	}
	renwin->AddRenderer(ren);
	renwin->SetInteractor(iren);
	renwin->SetSize(1280,800);
	iren->Initialize();
	datawin->AddRenderer(dataren);
	datawin->SetInteractor(data_iren);
	datawin->SetSize(1920	,1200);

	data_iren->Initialize();
	
	initializeLights();
	double initialbounds [6] = {1,-1,1,-1,1,-1};
	ren->ResetCamera(initialbounds);
	dataren->ResetCamera(initialbounds);
	if (useTNG)
		initializeTNG();
	if (useTracker)
		initializeTracker(); 
	
	   
	
	//Add timer callback
	vtkSmartPointer<UpdateData> updateCallback = vtkSmartPointer<UpdateData>::New();
	iren->AddObserver(vtkCommand::TimerEvent, updateCallback);
	iren->CreateRepeatingTimer(TIMER_LENGTH); 
	
	if (drawKinect)
	{
		devMan->GetCameraDataForAllDevices(cameraDataVector); 
	}
	updatePolyData();  
	
	/*ren->GetActiveCamera()->SetPosition(0,0,0.3); 
	ren->GetActiveCamera()->Roll(180.0);
	ren->GetActiveCamera()->Azimuth(180.0);

	ren->GetActiveCamera()->Modified();    */
	printf("rencamdis %f",dataren->GetActiveCamera()->GetDistance());
	printf("renviewangle %f",dataren->GetActiveCamera()->GetViewAngle()); 
	printf("camdis %f",dataren->GetActiveCamera()->GetDistance());
	printf("viewangle %f",dataren->GetActiveCamera()->GetViewAngle());
	 

	// Continue  interacting
  MSG msg;
    while (1) {
        PeekMessage(&msg, NULL, 0, 0, PM_REMOVE);
        TranslateMessage(&msg);
        DispatchMessage(&msg);
       
    }
	if (drawKinect)
		delete devMan;
	
	return 0;
}
