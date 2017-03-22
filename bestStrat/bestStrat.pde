/*
 * ---------- Binding the code with call javascript/serial port ----------
 */

private PLabBridge pBridge;

private String received = null;

void bindPLabBridge (PLabBridge bridge) {

  pBridge = bridge;
  
  // Vi trenger noe som hoerer etter endringer i verdi
  // We need something that listens to a change in the value
  // Vi bruker en anonym indre klasse for dette
  // We use an anonymous inner class for this
  bridge.subscribeMessages (new PLabRecv() {
    public void receive (int integer) {
      // code on BT receive goes here.
    }
  });
  size(bridge.getWidth(), bridge.getHeight());
}

void btWrite(int integer) {
  if (pBridge != null) {
    pBridge.send(integer);
  }
}


//-------------
//
//  Minimal Processing GUI library.
//  PLab 2015
//
//  Dag Svanæs, IDI, NTNU.
// 

int[] white = {255,255,255};
int[] black = {0,0,0};
int[] red = {255,0,0};
int[] green = {0,255,0};
int[] blue = {0,0,255};
int[] yellow = {255,255,0};
int[] gray = {128,128,128};

float scaleOfGUI = 1.0;
int canvasWidth = 0, canvasHeight = 0;

void setCanvas(int w, int h) {
  canvasWidth = w;
  canvasHeight = h;
 // size(w,h);
}

void scaleGUI() {
   if (pBridge != null) {
     scaleOfGUI = pBridge.getWidth() / canvasWidth;
     scale(scaleOfGUI);
   };
}

boolean inside(int x,int y, int x0,int y0,int w,int h) {
  return (((x >= x0) && (x < (x0+w))) && 
      ((y >= y0) && (y < (y0+h))));
}

boolean mouseInside(int[] rect) {
  int mouseXScaled = int(mouseX / scaleOfGUI);
  int mouseYScaled = int(mouseY / scaleOfGUI);
  return inside(mouseXScaled, mouseYScaled, rect[0],rect[1],rect[2],rect[3]);
}

void drawButton(int[] xywh, String buttonText) {
    boolean mousePressedInButton = mouseInside(xywh) && mousePressed;
    stroke(0);
    if (mousePressedInButton) {
      fill(0); }
    else {
      fill(255); 
    } 
    rect(xywh[0],xywh[1],xywh[2],xywh[3],10);
    if (mousePressedInButton) {
      fill(255); }
    else {
      fill(0); 
    }   
    int tSize = xywh[3] * 3 / 5;
    textSize(tSize);
    textAlign(CENTER, CENTER);
    text(buttonText,xywh[0]+xywh[2]/2,xywh[1]+xywh[3]/2-2);
}

void drawText(int[] xywh, String buttonText) {
    stroke(0);
    fill(255);  
    rect(xywh[0],xywh[1],xywh[2],xywh[3]);
    fill(0);  
    int tSize = xywh[3] * 3 / 5;
    textSize(tSize);
    textAlign(LEFT, CENTER);
    text(buttonText,xywh[0]+2,xywh[1]+xywh[3]/2-2);
}
  
void drawRectangle(int[] xywh, int[] drawColor) {
    stroke(0);
    fill(drawColor[0],drawColor[1],drawColor[2]);  
    rect(xywh[0],xywh[1],xywh[2],xywh[3]);
}

void drawCircle(int[] xyr, int[] drawColor) {
    stroke(0);
    fill(drawColor[0],drawColor[1],drawColor[2]);  
    ellipseMode(RADIUS);
    ellipse(xyr[0],xyr[1],xyr[2],xyr[2]);
}

/*
* ---------- End of library  -------------
*/
//------------------------------------------------------------------------------------- 
//-------------------------------------------------

int[] buttonFrameForward = {150,25,100,25};
String buttonStringForward = "Forward";


int[] buttonFrameBackward = {150,225,100,25};
String buttonStringBackward = "Backward";

boolean bluetoothActive = false;

void setup() {
  // setupSerial();              // Uncomment this to test with bluetooth from PC/Mac...
  size(400,600);
  setCanvas(400,600);              // Canvas size is 400 x 600 pixels.
  background(128);            // Background color is gray (128,128,128).
  stroke(0);                  // Stroke color is black (0,0,0)
}

void btRead(int integer) {
  println("Received: " + integer);
}
  
void draw() {
  scaleGUI();
  drawButton(buttonFrameForward,buttonStringForward);
  drawButton(buttonFrameBackward,buttonStringBackward);
}

void mousePressed() {
   if (mouseInside(buttonFrameForward)) {
      btWrite(1);
   } 
     else if (mouseInside(buttonFrameBackward)) {
      btWrite(0);  
   }   
}