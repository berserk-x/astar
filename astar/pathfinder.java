import java.awt.*;
import java.awt.event.*;
import java.awt.Graphics;
import javax.swing.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import java.io.*;
import java.net.Socket;
import java.util.Optional;
import java.util.ArrayList;

public class pathfinder extends JFrame
{
    public static boolean showtrace = true; // does not erase image
    BufferedImage diamondgif, chargif;  // animated gif images
    Graphics display;
    Image dbuf; // double buffer
    static int gap = 24; // 24;  // side/radius of hexagon
    static int yoff = 40; // 40;
    static int delaytime = 250; // animation delay time
    astar_base PG;
    int rows, cols;
    int XDIM, YDIM; // window dimensions
    int gemx, gemy, profx, profy;
    BufferedImage[] imageof; // image vector for terrain.
    BufferedImage[] imagechar; //image vector for character based on terrain.
    Color[] colorof ={Color.green,Color.yellow,Color.white,Color.blue};
 // color corresponding to each terrain type.

    public static String fpath = "";  // file path prefix
    public static String savefile = "myastar.run";
    public static boolean downloadconfig = false;
    public static boolean sendpath = false;
    public static boolean loadcosts = true; // load costs from file/socket

    // graphical representation
    Hexagon[][] HX;  // array of hexagons
    int hpdist = Hexagon.calchpdist(gap);
    
    // access center graphical coordinates at cell i,j
    int getx(int i, int j) { return HX[i][j].x; }
    int gety(int i, int j) { return HX[i][j].y; }

    Graphics Gg;

    public pathfinder(int r, int c, int g, String[] argv) // constructor
    {
	rows = r; cols = c; gap=g;
	if (argv.length>0 && !isint(argv[0])) {
	    astar_base.genmap=false; // load map from file or socket
	    if (argv.length==2&&argv[0].equals("download"))
		{downloadconfig=sendpath=true;}
	}
	PG = new myastar(rows,cols);  // note it's myastar, not astar
	if (downloadconfig) downloadrun(defaultserver,defaultsrvport,argv[1]);
	else if (!astar_base.genmap&&!argv[0].equals("relay")) loadrun(argv[0],argv.length==1);  // argv.length==1 means load cost vector as well.
	else if (!astar_base.genmap) loadrun(true); // relayconfig set externally
	if (astar_base.genmap) PG = new myastar(r,c); // if (down)loadrun failed
	rows = PG.ROWS;  cols = PG.COLS;
	System.out.print("Costs: ");
	for(int tc:PG.costof) System.out.print(tc+" ");
	System.out.println();
	hpdist = Hexagon.calchpdist(gap);	
	HX = new Hexagon[rows][cols];  // graphical map
	for(int i=0;i<rows;i++)
	    { 
	      int odd = i%2;
	      for(int j=0;j<cols;j++)
		{
		    HX[i][j] = new Hexagon(yoff/2+(j*2+odd)*hpdist,yoff+gap+(3*gap/2*i),gap);
		}
	    }//for i,j
	XDIM = cols*hpdist*2+ yoff/2; 
	YDIM = ((rows+1)*gap*3)/2 - yoff/4;
	this.setBounds(0,0,XDIM+5,YDIM+yoff+5);
        //this.setLocationRelativeTo(null);
	this.setVisible(true);
        this.toFront();
        this.repaint();
        this.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
 try {   // draw to double buffer
	dbuf = createImage(XDIM,YDIM+yoff);
	display = dbuf.getGraphics();  // display draws to buffer
	display.fillRect(0,0,XDIM,YDIM+yoff);  // fill background	

	diamondgif = ImageIO.read(new File(fpath+"gem1.gif"));
	prepareImage(diamondgif,this);
	chargif = ImageIO.read(new File(fpath+"man15.gif"));
	prepareImage(chargif,this);
	
	imagechar = new BufferedImage[4]; // image of character on terrain type
	imagechar[astar_base.OPEN] = chargif;
	imagechar[astar_base.WATER] =
	    ImageIO.read(new File(fpath+"boat.gif"));
	imagechar[astar_base.FIRE] = 
	    ImageIO.read(new File(fpath+"dragon.gif"));
	imagechar[astar_base.DESERT] = chargif;
	prepareImage(imagechar[astar_base.WATER],this);
	imageof = new BufferedImage[4];  // background image for terrain type
	BufferedImage im = ImageIO.read(new File(fpath+"Water.gif"));
	int iw = im.getWidth(this), ih = im.getHeight(this);
	imageof[astar_base.WATER]= new BufferedImage(gap*2,gap*2,BufferedImage.TYPE_INT_RGB);
	Hexagon Hi = new Hexagon(hpdist,gap,gap);	
	Gg = imageof[astar_base.WATER].getGraphics();
	//	Gg.setClip(Hi);
	Gg.drawImage(im,0,0,null);
	prepareImage(imageof[astar_base.WATER],this);

	BufferedImage imm = ImageIO.read(new File(fpath+"grass1.gif"));	
	iw = imm.getWidth(null); ih = imm.getHeight(null);
	imageof[astar_base.OPEN] = new BufferedImage(gap*2,gap*2,BufferedImage.TYPE_INT_RGB);
	Gg = imageof[astar_base.OPEN].getGraphics();
	//	Gg.setClip(Hi);
	Gg.drawImage(imm,0,0,null);
	prepareImage(imageof[astar_base.OPEN],this);

	//	BufferedImage imf = ImageIO.read(new File(fpath+"volcano.gif"));
	BufferedImage imf = ImageIO.read(new File(fpath+"flames.jpeg"));	
	iw = imf.getWidth(null); ih = imf.getHeight(null);
	imageof[astar_base.FIRE] = new BufferedImage(gap*2,gap*2,BufferedImage.TYPE_INT_RGB);
	Gg = imageof[astar_base.FIRE].getGraphics();
	Gg.drawImage(imf,0,0,null);
	prepareImage(imageof[astar_base.FIRE],this);	

	try{Thread.sleep(500);} catch(Exception e) {} // Synch with system
	// draw static background as a green rectangle
	display.setColor(Color.green);
	display.fillRect(0,0,XDIM,YDIM+yoff);  // fill background

	// generate random starting positions.
	  // generate initial positions of professor and diamond
	if (astar_base.genmap)
	{
	  do 
	    {
		gemx = (int)(Math.random() * PG.COLS);  // diamond
		gemy = (int)(Math.random() * PG.ROWS);
	    }
	  while (PG.Map[gemy][gemx]!=PG.OPEN);
	  do 
	    {
		profx = (int)(Math.random() * PG.COLS);  
                profy = (int)(Math.random() * PG.ROWS);
	    }
	  while (PG.Map[profy][profx]!=PG.OPEN ||
	       astar_base.hexdist(gemy,gemx,profy,profx)<PG.ROWS*5/5);
	}// if need to generate new starting positions

	// draw map
	drawmap();
	// draw professor and diamond, initial position

	// draw static map
	display = (Graphics)this.getGraphics(); // change display
	display.drawImage(dbuf,0,0,this);
	
	int px = getx(profy,profx), py = gety(profy,profx); // center hx coords
	display.drawImage(imagechar[PG.Map[profy][profx]],
			  (px-gap/2),(py-gap/2),gap,gap,null);
	px = getx(gemy,gemx); py = gety(gemy,gemx);
	display.drawImage(diamondgif,px-gap/2,py-gap/2,gap,gap,null);
	/*	
	display.drawImage(imagechar[PG.Map[profy][profx]],
			  (profx*gap),(profy*gap)+yoff,gap,gap,null);
	display.drawImage(diamondgif,gemx*gap,gemy*gap+yoff,gap,gap,null);

	animate();
	if (astar_base.genmap || downloadconfig)
	    { saverun(); // save the run
	      System.out.println("run saved in "+(fpath+savefile));
	    }
        */  
 }//try
 catch (Exception eee) {}
    } // constructor

    public void run()
    {
	animate();
	if (astar_base.genmap || downloadconfig || !loadcosts)
	    { saverun(); // save the run
	      System.out.println("run saved in "+(fpath+savefile));
	    }
    }

    // save map and positions to file:
    void saverun()
    {
	try {
	    mapconfig mc = new mapconfig(profy,profx,gemy,gemx,PG.Map,PG.costof);
	    ObjectOutputStream dout = new ObjectOutputStream(new FileOutputStream(fpath+savefile));
	    dout.writeObject(mc);
	    dout.close();
	}
	catch(IOException ie) {System.out.println(ie);}
    }//saverun

    void uploadrun(ObjectOutputStream bout) throws IOException
    { 
	mapconfig mc = new mapconfig(profy,profx,gemy,gemx,PG.Map,PG.costof);
	bout.writeObject(mc);
    }//uploadrun

    void loadrun(String filen, boolean loadcost)
    {
        loadcosts = loadcost;
	try {
	    ObjectInputStream din = new ObjectInputStream(new FileInputStream(fpath+filen));
	    mapconfig mc = (mapconfig)din.readObject();
	    din.close();
	    profy = mc.chary;  profx = mc.charx;
	    gemy = mc.goaly;  gemx = mc.goalx;
	    PG.genmap=false;
	    PG.ROWS = mc.ROWS;  PG.COLS = mc.COLS;
	    PG.Map = mc.Map;
	    if (loadcost) PG.costof = mc.costof;
	}
	catch(Exception ie) {System.out.println(ie); astar_base.genmap=true;}
    }//loadrun
    static mapconfig relayconfig=null;
    void loadrun(boolean loadcost)
    {   if (relayconfig==null) {astar_base.genmap=true; return;}
	mapconfig mc=relayconfig;
	profy = mc.chary;  profx = mc.charx;
	gemy = mc.goaly;  gemx = mc.goalx;
	PG.genmap=false;
	PG.ROWS = mc.ROWS;  PG.COLS = mc.COLS;
	PG.Map = mc.Map;
	if (loadcost) PG.costof = mc.costof;
    }//loadrun	

    void downloadrun(String srvaddr, int srvport, String cname)
    {	   
	try {
	    csk = new Socket(srvaddr,srvport);
	    csk.setSoTimeout(5000);
	    cskout = new ObjectOutputStream(csk.getOutputStream());
	    cskout.writeObject(cname);
	    cskin = new ObjectInputStream(csk.getInputStream());
	    mapconfig mc = (mapconfig)cskin.readObject();
	    profy = mc.chary;  profx = mc.charx;
	    gemy = mc.goaly;  gemx = mc.goalx;
	    PG.genmap=false;
	    PG.ROWS = mc.ROWS;  PG.COLS = mc.COLS;
	    PG.Map = mc.Map;
	    PG.costof = mc.costof;	    
	}catch (Exception e) {
	    System.out.println(e);
	    astar_base.genmap=true; downloadconfig=false;
	}
    }//downloadrun

    public void trace(int y, int x, char t)
    {
	if (x<0 || x>=cols || y<0 || y>=rows) return;
	display.setColor(Color.black);	
	display.drawString(""+t,getx(y,x)-gap,gety(y,x)-gap-4);
    }

    public void animate() {
	// invert path.
	Optional<coord> pathopt = Optional.empty();
	try {  // student code
	   pathopt = PG.search(gemy,gemx, profy,profx); // call SEARCH here
	} catch (Exception e) {System.out.println("error in call to search: "+e);}
        // if search crashed then pathopt stays empty.
	if (sendpath && csk!=null) try {
		cskout.writeObject(toList(pathopt));
	    } catch (Exception e) {System.out.println(e);}

	if (pathopt.isEmpty()) 
	    {   display.setColor(Color.red);
		display.drawString("NO PATH TO TARGET!",50,100); 
		System.out.println("no path"); 
		if (sendpath && csk!=null) try {
	  	  String response = (String)cskin.readObject();
          	  System.out.println("Response from server: "+response);
          	  csk.close();
		}catch(Exception ie){}
		return;
	    }
        coord path = pathopt.get(); // checked isEmpty above
	int px=0, py=0; // for calculating graphical coords
	while (path.parent.isPresent()) //not at end of path
	    {   coord parent = path.parent.get(); // just checked above
		px = getx(path.y,path.x); py = gety(path.y,path.x);
		display.drawImage(imagechar[PG.Map[path.y][path.x]],
				(px-gap/2),(py-gap/2),gap,gap,null);
		//	      display.drawImage(imagechar[PG.Map[path.y][path.x]],
		//				(path.x*gap),(path.y*gap)+yoff,gap,gap,null);

		//System.out.printf("%d,%d: %d\n",path.y,path.x,PG.Map[path.y][path.x]);

	      try{Thread.sleep(delaytime);} catch(Exception se) {}
	      //	      display.drawImage(imageof[PG.Map[path.y][path.x]],
	      //				(path.x*gap),(path.y*gap)+yoff,gap,gap,null);
	      //	      display.setColor(Color.red);
	      //      	      display.fillOval((path.x*gap)+8,(path.y*gap)+yoff+8,4,4);
	      // for animation:
	      //	      display.drawImage(diamondgif,gemx*gap,gemy*gap+yoff,gap,gap,null);	      

	      if (!showtrace) // erase trail - redraw static map
		  display.drawImage(dbuf,0,0,this);
	      path = parent;
	    }//with path not at terminus
	px = getx(gemy,gemx); py = gety(gemy,gemx);
	display.drawImage(diamondgif,px-gap/2,py-gap/2,gap,gap,null);
	display.drawImage(imagechar[PG.Map[gemy][gemx]],px-gap/2,py-gap/2,gap,gap,null);

	if (sendpath && csk!=null) try {
	  String response = (String)cskin.readObject();
          System.out.println("Response from server: "+response);
          csk.close();
	}catch(Exception ie){}

    }//animate

    public void drawmap()
    {   
	int i, j;
	for(i=0;i<PG.ROWS;i++)
	    for(j=0;j<PG.COLS;j++)
		{

		    display.setColor(colorof[ PG.Map[i][j] ]);
		    //		    display.fillPolygon(HX[i][j]);
		    //		    Gg = imageof[PG.Map[i][j]].getGraphics();
		    //		    Gg.setClip(HX[i][j]);
		    //		    Gg.drawImage(imageof[PG.Map[i][j]],0,0,null);
		    //		    display.drawImage(imageof[PG.Map[i][j]],j*gap,(i*gap)+yoff,gap,gap,null);
		    display.setClip(HX[i][j]);
		    display.drawImage(imageof[PG.Map[i][j]],yoff/2 + (j*2+(i%2)-1)*hpdist,(3*gap/2*i)+yoff,null);		    
		}
	//try{Thread.sleep(1000);} catch(Exception e) {} 
    } // drawmap

    public static boolean isint(String s) // check if string is an integer
    {   boolean answer=true;
	try {Integer.parseInt(s);}
	catch (NumberFormatException nfe) {answer=false;}
	return answer;
    }

    //convert coord path to ArrayList for socket comm:
    // use a single 32-bit Integer to encode y-x coords
    // first value in list is known dist of entire path
    ArrayList<Integer> toList(Optional<coord> popt) {
        ArrayList<Integer> path = new ArrayList<Integer>(rows+cols/2);
        popt.ifPresent(current -> {
           path.add(current.knowncost);
           boolean stop = false;
           while (!stop) {
              int yx = current.y << 16;
              yx += current.x;
              path.add(yx);
              if (current.parent.isPresent()) current = current.parent.get();
              else stop = true;
           }//while
        });
        return path;
    }//toList

    static int defaultrows = 24;
    static int defaultcols = 36;
    static String defaultserver = "10.25.184.13"; //"127.0.0.1";
    static int defaultsrvport = 17003;
    protected Socket csk;
    protected ObjectInputStream cskin;
    protected ObjectOutputStream cskout;
    public static void main(String[] args)
    {
        int r = defaultrows, c = defaultcols, g = gap;
        if (args.length>1)
         try
            {
                r = Integer.parseInt(args[args.length-2]);
                c = Integer.parseInt(args[args.length-1]);
                //g = Integer.parseInt(args[args.length-1]);                
            }
         catch (NumberFormatException nfe) {r=defaultrows; c=defaultcols;}
        if (r<1) r=defaultrows;
        if (c<1) c = defaultcols;
        pathfinder pf = new pathfinder(r,c,g,args);
        pf.run();
    }//main
} // class pathfinder

