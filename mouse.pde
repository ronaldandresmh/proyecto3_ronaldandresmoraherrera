// mouse, by lmcapacho
import processing.serial.*;

Maze maze;

// El puerto serial
//Serial myPort;

void setup() {

  // Tamaño de la ventana
  size(400, 400);

  // Tamaño fuente
  textSize(20);
  // Color de fondo de la ventana 
  background(51);
  // Alineación del texto
  textAlign(CENTER, CENTER);

  // Crea una nueva cuadrícula de 6 x 6   
  maze = new Maze(6);
  // Abre el puerto serial con velocidad de 9600 baudios
  //myPort = new Serial(this, "/dev/ttyUSB1", 9600);
  //myPort.clear();
  
  // Ejemplo: Coloca pared en norte y este de la celda 2,2 
  maze.setWalls(0,0, Maze.NORTH | Maze.EAST | Maze.WEST );
  maze.setWalls(0,1, Maze.EAST | Maze.WEST );
  maze.setWalls(0,2, Maze.WEST | Maze.SOUTH );
  maze.setWalls(0,3, Maze.NORTH | Maze.EAST | Maze.WEST );
  maze.setWalls(0,4, Maze.WEST );
  maze.setWalls(0,5, Maze.SOUTH | Maze.EAST | Maze.WEST );
  maze.setWalls(1,0, Maze.NORTH | Maze.EAST | Maze.WEST );
  maze.setWalls(1,1, Maze.EAST | Maze.WEST );
  maze.setWalls(1,2, Maze.EAST );
  maze.setWalls(1,3, Maze.SOUTH | Maze.WEST );
  maze.setWalls(1,4, Maze.NORTH );
  maze.setWalls(1,5, Maze.SOUTH | Maze.WEST );
  maze.setWalls(2,0, Maze.NORTH |  Maze.WEST );
  maze.setWalls(2,1, Maze.SOUTH | Maze.WEST );
  maze.setWalls(2,2, Maze.NORTH |  Maze.WEST );
  maze.setWalls(2,3, Maze.SOUTH);
  maze.setWalls(2,4, Maze.NORTH | Maze.EAST | Maze.SOUTH );
  maze.setWalls(2,5, Maze.NORTH | Maze.SOUTH);
  maze.setWalls(3,0, Maze.NORTH | Maze.SOUTH);
  maze.setWalls(3,1, Maze.NORTH | Maze.SOUTH);
  maze.setWalls(3,2, Maze.NORTH  );
  maze.setWalls(3,3, Maze.EAST );
  maze.setWalls(3,4, Maze.EAST | Maze.WEST );
  maze.setWalls(3,5, Maze.SOUTH );
  maze.setWalls(4,0, Maze.NORTH | Maze.SOUTH);
  maze.setWalls(4,1, Maze.NORTH );
  maze.setWalls(4,2, Maze.EAST | Maze.SOUTH );
  maze.setWalls(4,3, Maze.NORTH | Maze.WEST );
  maze.setWalls(4,4, Maze.EAST | Maze.WEST );
  maze.setWalls(4,5, Maze.EAST | Maze.SOUTH );
  maze.setWalls(5,0, Maze.NORTH | Maze.EAST | Maze.SOUTH );
  maze.setWalls(5,1, Maze.NORTH | Maze.EAST | Maze.SOUTH);
  maze.setWalls(5,2, Maze.NORTH | Maze.EAST | Maze.WEST );
  maze.setWalls(5,3, Maze.EAST  );
  maze.setWalls(5,4, Maze.EAST | Maze.WEST );
  maze.setWalls(5,5, Maze.SOUTH | Maze.EAST | Maze.WEST );
  
  

  
}

int a,i;
void draw() { 
  maze.display();
  maze.setMouse(a,i);
   a=a+1;
   delay(200);
   if(a==5){
   a=0;
   i=i+1;
   if(i==5){
   
   i=0;
   }
   }
  /*while (myPort.available() > 0) {
    byte[] inBuffer = new byte[11];
    if( myPort.readBytesUntil('\n', inBuffer) > 0){
      String myString = new String(inBuffer);
      print(int(myString.substring(0,3)));
      print(int(myString.substring(3,6)));
      println(int(myString.substring(6,9)));
    }  
  }*/
}