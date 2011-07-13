﻿package {	import flash.display.BitmapData;	import flash.display.Bitmap;	import flash.display.DisplayObject;	import flash.geom.Rectangle;	import flash.geom.Point;	import flash.geom.ColorTransform;	import flash.utils.ByteArray;	//as3kinect (OpenKinect) libraries	import org.as3kinect.as3kinect;	import org.as3kinect.as3kinectWrapper;	import org.as3kinect.as3kinectUtils;	import org.as3kinect.events.as3kinectWrapperEvent;	import org.as3kinect.objects.motorData;	import flash.events.MouseEvent;	import flash.events.Event;	import flash.events.TouchEvent;	import flash.display.MovieClip;	import Date;	import User;	import Animal;	import flash.events.TimerEvent;    import flash.utils.Timer;	public class TheAbyss extends MovieClip	{		//My variables		private var users:Array;		private var animals:Array;		private var bg:MovieClip;		private var date:Date;		private var depthTimer:Timer;		//Instanciating the wrapper library		private var as3w:as3kinectWrapper = new as3kinectWrapper();		//Canvas variables		private var canvas_depth:BitmapData;		private var bmp_depth:Bitmap;		private var canvas_video:BitmapData;		private var bmp_video:Bitmap;		private var blobarray:Array;		public function TheAbyss()		{			super();			users = new Array;			animals = new Array;			bg = new MovieClip;			date = new Date();			//Add as3kinectWrapper events (depth, video and acceleration data)			as3w.addEventListener(as3kinectWrapperEvent.ON_DEPTH, gotdepth);			as3w.addEventListener(as3kinectWrapperEvent.ON_VIDEO, gotvideo);			as3w.addEventListener(as3kinectWrapperEvent.ON_ACCELEROMETER, gotmotordata);			//Add depth BitmapData to depthcam MovieClip			as3w.depth.mirrored = true;			canvas_depth = as3w.depth.bitmap;			bmp_depth = new Bitmap(canvas_depth);			depthcam.removeChildAt(0);			depthcam.addChild(bmp_depth);			//Add video BitmapData to rgbcam MovieClip			as3w.video.mirrored = true;			canvas_video = as3w.video.bitmap;			bmp_video = new Bitmap(canvas_video);			rgbcam.removeChildAt(0);			rgbcam.addChild(bmp_video);			//On every frame call the update method			this.addEventListener(Event.ENTER_FRAME, update);			users[0] = new User(320,240,50,100,100,date.getTime());			addChild(users[0].dot);			populateAnimals();		}		private function populateAnimals():void		{			for (var i=0; i < 5; i++)			{				var a = new angler();				a.x = a.width*2*(i);				a.y = 100;				a.stop();				animals.push(a);				addChild(a);			}		}		//UPDATE METHOD (This is called each frame)		private function update(event:Event)		{			as3w.video.getBuffer();			as3w.motor.getData();			as3w.depth.getBuffer();			animateAnimals();		}		//GOT DEPTH METHOD		private function gotdepth(event:as3kinectWrapperEvent):void		{			//Convert Received ByteArray into BitmapData			as3kinectUtils.byteArrayToBitmapData(event.data, canvas_depth);			//Transform to Black / White image for blob detection;			var tmpDepth:BitmapData = canvas_depth.clone();			as3w.depth.minDistance = 400;			as3w.depth.maxDistance = 900;			for (var depth = 20; depth <= 90; depth+=5)			{				as3kinectUtils.setBlackWhiteFilter(tmpDepth, (depth/100)* 256);				//Process Blobs from image;				blobarray = as3kinectUtils.getBlobs(tmpDepth);				//Send pointers somwhere we cant see them				/*				for(var i = 0, max = pointer.length; i < max; i++){				//pointer[i].x = -10;				}				*/				//Create pointers if not enough and position them in the bottom center of the blob rect				var unique:int;				var u:User;				for (var i = 0, max = blobarray.length; i < max; i++)				{					date = new Date;					u = new User(blobarray[i].point.x,blobarray[i].point.y,depth,blobarray[i].width,blobarray[i].height,date.getTime());					/*					unique = u.checkUnique(users);					if( unique == -1){						users[0].updateTo(u);							//animals[i].follow(u);						addChild(u.dot);					}					else{						users.push(u);						trace('good spot');					}					*/					users[0].updateTo(u);					addChild(users[0].dot);				}			}		}				private function animateAnimals(){			for(var i = 0; i < animals.length; i++){				if( i < users.length ){					if( users[i].updated ){						animals[i].follow(users[i]);						users[i].updated = false;						trace('follow');					}				}				else					animals[i].default_swim();			}		}		//GOT VIDEO METHOD		private function gotvideo(event:as3kinectWrapperEvent):void		{			//Convert Received ByteArray into BitmapData			as3kinectUtils.byteArrayToBitmapData(event.data, canvas_video);		}		//GOT MOTOR DATA (Accelerometer info);		private function gotmotordata(event:as3kinectWrapperEvent):void		{			var object:motorData = event.data;			info.text = "raw acceleration:\n\tax: " + object.ax + "\n\tay: " + object.ay;			info.appendText("\n\taz: " + object.az + "\n\n");			info.appendText("mks acceleration:\n\tdx: " + object.dx + "\n\tdy: " + object.dy);			info.appendText("\n\tdz: " + object.dz + "\n");		}		/*		private function touching(cur, i):void{		var curX:int = cur.x;		var curY:int = cur.y;				curX = (curX / 640) * stage.width;		curY = (curY / 480) * stage.height;				trace(curX+', '+curY);				if(curX> prevX[i]){		cur.gotoAndStop(10);		}		else if(curX < prevX[i]){		cur.gotoAndStop(30);		}		cur.x = curX;		cur.y = curY;		prevX[i] = curX;		prevY[i] = curY;		}		*/	}}