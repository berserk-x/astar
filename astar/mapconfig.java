public class mapconfig implements java.io.Serializable
{
    int charx, chary; // position of character (profx, profy)
    int goalx, goaly; // position of goal (gemx,gemy);
    int ROWS, COLS;   // dimensions of map
    int[][] Map;   // terrain
    int[] costof; // costof vector
    public mapconfig(int cy,int cx,int gy,int gx,int[][] M, int[] C)
    {
	chary=cy;  charx=cx;  goaly=gy;  goalx=gx;
	Map = M;
	ROWS = Map.length;
	COLS = Map[0].length;
	costof = C;
    }
}//mapconfig class
