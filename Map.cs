using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[, ,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -40;//CHANGE
        private double maxWorkspaceX = 40;
        private double minWorkspaceY = -40;
        private double maxWorkspaceY = 40;
        // New Class
        public class Zone
        {
            public double xmin, ymin, xmax, ymax;

            public Zone()
            {
            }

            public Zone(double x1, double x2, double y1, double y2)
            {
                xmin = x1;
                ymin = y1;
                xmax = x2;
                ymax = y2;
            }

            public double minx() { return xmin; }
            public double maxx() { return xmax; }
            public double miny() { return ymin; }
            public double maxy() { return ymax; }
        }
        public int numNThrowZ = 0;
        public List<Zone> mapNthrowZ;




        public Map()
        {

            // This is hard coding at its worst. Just edit the file to put in
            // segments of the environment your robot is working in. This is
            // used both for visual display and for localization.

            // ****************** Additional Student Code: Start ************
            #region drawMap
            // Change hard code here to change map:
            numMapSegments = 169;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];
            //assuming things are more or less symettric, and also that the distance between the bottom of the wall to the top has a horizontal distance of 0.07m
            mapSegmentCorners[0, 0, 0] = 13.945;
            mapSegmentCorners[0, 0, 1] = 13.794;
            mapSegmentCorners[0, 1, 0] = -7.945;
            mapSegmentCorners[0, 1, 1] = 13.794;

            mapSegmentCorners[1, 0, 0] = 1.225;
            mapSegmentCorners[1, 0, 1] = 11.0;
            mapSegmentCorners[1, 1, 0] = 1.225;
            mapSegmentCorners[1, 1, 1] = 8.26;

            mapSegmentCorners[2, 0, 0] = 4.775;
            mapSegmentCorners[2, 0, 1] = 11.0;
            mapSegmentCorners[2, 1, 0] = 4.775;
            mapSegmentCorners[2, 1, 1] = 8.26;

            mapSegmentCorners[3, 0, 0] = 4.775;
            mapSegmentCorners[3, 0, 1] = 11.0;
            mapSegmentCorners[3, 1, 0] = 10.565;
            mapSegmentCorners[3, 1, 1] = 11.0;

            mapSegmentCorners[4, 0, 0] = 1.225;
            mapSegmentCorners[4, 0, 1] = 8.26;
            mapSegmentCorners[4, 1, 0] = -1.825;
            mapSegmentCorners[4, 1, 1] = 8.26;

            mapSegmentCorners[5, 0, 0] = 4.775;
            mapSegmentCorners[5, 0, 1] = 8.26;
            mapSegmentCorners[5, 1, 0] = 7.825;
            mapSegmentCorners[5, 1, 1] = 8.26;

            mapSegmentCorners[6, 0, 0] = 1.225;
            mapSegmentCorners[6, 0, 1] = 11.0;
            mapSegmentCorners[6, 1, 0] = -4.565;
            mapSegmentCorners[6, 1, 1] = 11.0;

            mapSegmentCorners[7, 0, 0] = -4.565;
            mapSegmentCorners[7, 0, 1] = 11.0;
            mapSegmentCorners[7, 1, 0] = -4.565;
            mapSegmentCorners[7, 1, 1] = -4.06;

            mapSegmentCorners[8, 0, 0] = 5.515;
            mapSegmentCorners[8, 0, 1] = 5.95;
            mapSegmentCorners[8, 1, 0] = 0.485;
            mapSegmentCorners[8, 1, 1] = 5.95;

            mapSegmentCorners[9, 0, 0] = 5.515;
            mapSegmentCorners[9, 0, 1] = 0.93;
            mapSegmentCorners[9, 1, 0] = 0.485;
            mapSegmentCorners[9, 1, 1] = 0.93;

            mapSegmentCorners[10, 0, 0] = 0.485;
            mapSegmentCorners[10, 0, 1] = 5.95;
            mapSegmentCorners[10, 1, 0] = 0.485;
            mapSegmentCorners[10, 1, 1] = 0.93;

            mapSegmentCorners[11, 0, 0] = 5.515;
            mapSegmentCorners[11, 0, 1] = 5.95;
            mapSegmentCorners[11, 1, 0] = 5.515;
            mapSegmentCorners[11, 1, 1] = 0.93;

            mapSegmentCorners[12, 0, 0] = 13.945;
            mapSegmentCorners[12, 0, 1] = 13.794;
            mapSegmentCorners[12, 1, 0] = 13.945;
            mapSegmentCorners[12, 1, 1] = -7.9;

            mapSegmentCorners[13, 0, 0] = -7.945;
            mapSegmentCorners[13, 0, 1] = 13.794;
            mapSegmentCorners[13, 1, 0] = -7.945;
            mapSegmentCorners[13, 1, 1] = -7.9;

            mapSegmentCorners[14, 0, 0] = -1.825;
            mapSegmentCorners[14, 0, 1] = 8.26;
            mapSegmentCorners[14, 1, 0] = -1.825;
            mapSegmentCorners[14, 1, 1] = -1.32;

            mapSegmentCorners[15, 0, 0] = 7.825;
            mapSegmentCorners[15, 0, 1] = 8.26;
            mapSegmentCorners[15, 1, 0] = 7.825;
            mapSegmentCorners[15, 1, 1] = -1.32;

            mapSegmentCorners[16, 0, 0] = 1.225;
            mapSegmentCorners[16, 0, 1] = -1.32;
            mapSegmentCorners[16, 1, 0] = -1.825;
            mapSegmentCorners[16, 1, 1] = -1.32;

            mapSegmentCorners[17, 0, 0] = 1.225;
            mapSegmentCorners[17, 0, 1] = -1.32;
            mapSegmentCorners[17, 1, 0] = 1.225;
            mapSegmentCorners[17, 1, 1] = -4.06;

            mapSegmentCorners[18, 0, 0] = 1.225;
            mapSegmentCorners[18, 0, 1] = -4.06;
            mapSegmentCorners[18, 1, 0] = -4.565;
            mapSegmentCorners[18, 1, 1] = -4.06;

            mapSegmentCorners[19, 0, 0] = 4.775;
            mapSegmentCorners[19, 0, 1] = -1.32;
            mapSegmentCorners[19, 1, 0] = 7.825;
            mapSegmentCorners[19, 1, 1] = -1.32;

            mapSegmentCorners[20, 0, 0] = 4.775;
            mapSegmentCorners[20, 0, 1] = -1.32;
            mapSegmentCorners[20, 1, 0] = 4.775;
            mapSegmentCorners[20, 1, 1] = -4.06;

            mapSegmentCorners[21, 0, 0] = 4.775;
            mapSegmentCorners[21, 0, 1] = -4.06;
            mapSegmentCorners[21, 1, 0] = 7.825;
            mapSegmentCorners[21, 1, 1] = -4.06;

            mapSegmentCorners[22, 0, 0] = 2.85;
            mapSegmentCorners[22, 0, 1] = -11.02;
            mapSegmentCorners[22, 1, 0] = 3.15;
            mapSegmentCorners[22, 1, 1] = -11.02;

            mapSegmentCorners[23, 0, 0] = 2.85;
            mapSegmentCorners[23, 0, 1] = -11.32;
            mapSegmentCorners[23, 1, 0] = 3.15;
            mapSegmentCorners[23, 1, 1] = -11.32;

            mapSegmentCorners[24, 0, 0] = 2.85;
            mapSegmentCorners[24, 0, 1] = -11.02;
            mapSegmentCorners[24, 1, 0] = 2.85;
            mapSegmentCorners[24, 1, 1] = -11.32;

            mapSegmentCorners[25, 0, 0] = 3.15;
            mapSegmentCorners[25, 0, 1] = -11.02;
            mapSegmentCorners[25, 1, 0] = 3.15;
            mapSegmentCorners[25, 1, 1] = -11.32;

            mapSegmentCorners[26, 0, 0] = 2.85;
            mapSegmentCorners[26, 0, 1] = -14.17;
            mapSegmentCorners[26, 1, 0] = 3.15;
            mapSegmentCorners[26, 1, 1] = -14.17;

            mapSegmentCorners[27, 0, 0] = 2.85;
            mapSegmentCorners[27, 0, 1] = -14.47;
            mapSegmentCorners[27, 1, 0] = 3.15;
            mapSegmentCorners[27, 1, 1] = -14.47;

            mapSegmentCorners[28, 0, 0] = 2.85;
            mapSegmentCorners[28, 0, 1] = -14.17;
            mapSegmentCorners[28, 1, 0] = 2.85;
            mapSegmentCorners[28, 1, 1] = -14.47;

            mapSegmentCorners[29, 0, 0] = 3.15;
            mapSegmentCorners[29, 0, 1] = -14.17;
            mapSegmentCorners[29, 1, 0] = 3.15;
            mapSegmentCorners[29, 1, 1] = -14.47;

            mapSegmentCorners[30, 0, 0] = -0.5;
            mapSegmentCorners[30, 0, 1] = -11.02;
            mapSegmentCorners[30, 1, 0] = -0.2;
            mapSegmentCorners[30, 1, 1] = -11.02;

            mapSegmentCorners[31, 0, 0] = -0.5;
            mapSegmentCorners[31, 0, 1] = -11.32;
            mapSegmentCorners[31, 1, 0] = -0.2;
            mapSegmentCorners[31, 1, 1] = -11.32;

            mapSegmentCorners[32, 0, 0] = -0.5;
            mapSegmentCorners[32, 0, 1] = -11.02;
            mapSegmentCorners[32, 1, 0] = -0.5;
            mapSegmentCorners[32, 1, 1] = -11.32;

            mapSegmentCorners[33, 0, 0] = -0.2;
            mapSegmentCorners[33, 0, 1] = -11.02;
            mapSegmentCorners[33, 1, 0] = -0.2;
            mapSegmentCorners[33, 1, 1] = -11.32;

            mapSegmentCorners[34, 0, 0] = -0.5;
            mapSegmentCorners[34, 0, 1] = -14.17;
            mapSegmentCorners[34, 1, 0] = -0.2;
            mapSegmentCorners[34, 1, 1] = -14.17;

            mapSegmentCorners[35, 0, 0] = -0.5;
            mapSegmentCorners[35, 0, 1] = -14.47;
            mapSegmentCorners[35, 1, 0] = -0.2;
            mapSegmentCorners[35, 1, 1] = -14.47;

            mapSegmentCorners[36, 0, 0] = -0.5;
            mapSegmentCorners[36, 0, 1] = -14.17;
            mapSegmentCorners[36, 1, 0] = -0.5;
            mapSegmentCorners[36, 1, 1] = -14.47;

            mapSegmentCorners[37, 0, 0] = -0.2;
            mapSegmentCorners[37, 0, 1] = -14.17;
            mapSegmentCorners[37, 1, 0] = -0.2;
            mapSegmentCorners[37, 1, 1] = -14.47;

            mapSegmentCorners[38, 0, 0] = 6.2;
            mapSegmentCorners[38, 0, 1] = -11.02;
            mapSegmentCorners[38, 1, 0] = 6.5;
            mapSegmentCorners[38, 1, 1] = -11.02;

            mapSegmentCorners[39, 0, 0] = 6.2;
            mapSegmentCorners[39, 0, 1] = -11.32;
            mapSegmentCorners[39, 1, 0] = 6.5;
            mapSegmentCorners[39, 1, 1] = -11.32;

            mapSegmentCorners[40, 0, 0] = 6.2;
            mapSegmentCorners[40, 0, 1] = -11.02;
            mapSegmentCorners[40, 1, 0] = 6.2;
            mapSegmentCorners[40, 1, 1] = -11.32;

            mapSegmentCorners[41, 0, 0] = 6.5;
            mapSegmentCorners[41, 0, 1] = -11.02;
            mapSegmentCorners[41, 1, 0] = 6.5;
            mapSegmentCorners[41, 1, 1] = -11.32;

            mapSegmentCorners[42, 0, 0] = 6.2;
            mapSegmentCorners[42, 0, 1] = -14.17;
            mapSegmentCorners[42, 1, 0] = 6.5;
            mapSegmentCorners[42, 1, 1] = -14.17;

            mapSegmentCorners[43, 0, 0] = 6.2;
            mapSegmentCorners[43, 0, 1] = -14.47;
            mapSegmentCorners[43, 1, 0] = 6.5;
            mapSegmentCorners[43, 1, 1] = -14.47;

            mapSegmentCorners[44, 0, 0] = 6.2;
            mapSegmentCorners[44, 0, 1] = -14.17;
            mapSegmentCorners[44, 1, 0] = 6.2;
            mapSegmentCorners[44, 1, 1] = -14.47;

            mapSegmentCorners[45, 0, 0] = 6.5;
            mapSegmentCorners[45, 0, 1] = -14.17;
            mapSegmentCorners[45, 1, 0] = 6.5;
            mapSegmentCorners[45, 1, 1] = -14.47;

            mapSegmentCorners[46, 0, 0] = 1.25;
            mapSegmentCorners[46, 0, 1] = -20.57;
            mapSegmentCorners[46, 1, 0] = -4.565;
            mapSegmentCorners[46, 1, 1] = -20.57;

            mapSegmentCorners[47, 0, 0] = 4.75;
            mapSegmentCorners[47, 0, 1] = -20.57;
            mapSegmentCorners[47, 1, 0] = 18.345;
            mapSegmentCorners[47, 1, 1] = -20.57;

            mapSegmentCorners[48, 0, 0] = 1.25;
            mapSegmentCorners[48, 0, 1] = -20.57;
            mapSegmentCorners[48, 1, 0] = 1.25;
            mapSegmentCorners[48, 1, 1] = -24.23;

            mapSegmentCorners[49, 0, 0] = 4.75;
            mapSegmentCorners[49, 0, 1] = -20.57;
            mapSegmentCorners[49, 1, 0] = 4.75;
            mapSegmentCorners[49, 1, 1] = -24.23;

            mapSegmentCorners[50, 0, 0] = -7.235;
            mapSegmentCorners[50, 0, 1] = -20.57;
            mapSegmentCorners[50, 1, 0] = -7.235;
            mapSegmentCorners[50, 1, 1] = -38.81;

            mapSegmentCorners[51, 0, 0] = 1.25;
            mapSegmentCorners[51, 0, 1] = -24.23;
            mapSegmentCorners[51, 1, 0] = -0.57;
            mapSegmentCorners[51, 1, 1] = -24.23;

            mapSegmentCorners[52, 0, 0] = -0.57;
            mapSegmentCorners[52, 0, 1] = -24.23;
            mapSegmentCorners[52, 1, 0] = -0.57;
            mapSegmentCorners[52, 1, 1] = -27.73;

            mapSegmentCorners[53, 0, 0] = -4.565;
            mapSegmentCorners[53, 0, 1] = -20.57;
            mapSegmentCorners[53, 1, 0] = -4.565;
            mapSegmentCorners[53, 1, 1] = -27.73;

            mapSegmentCorners[54, 0, 0] = -0.57;
            mapSegmentCorners[54, 0, 1] = -27.73;
            mapSegmentCorners[54, 1, 0] = -4.565;
            mapSegmentCorners[54, 1, 1] = -27.73;

            mapSegmentCorners[55, 0, 0] = -0.57;
            mapSegmentCorners[55, 0, 1] = -30.73;
            mapSegmentCorners[55, 1, 0] = -4.565;
            mapSegmentCorners[55, 1, 1] = -30.73;

            mapSegmentCorners[56, 0, 0] = -0.57;
            mapSegmentCorners[56, 0, 1] = -30.73;
            mapSegmentCorners[56, 1, 0] = -0.57;
            mapSegmentCorners[56, 1, 1] = -34.23;

            mapSegmentCorners[57, 0, 0] = -0.57;
            mapSegmentCorners[57, 0, 1] = -34.23;
            mapSegmentCorners[57, 1, 0] = 1.25;
            mapSegmentCorners[57, 1, 1] = -34.23;

            mapSegmentCorners[58, 0, 0] = 1.25;
            mapSegmentCorners[58, 0, 1] = -34.23;
            mapSegmentCorners[58, 1, 0] = 1.25;
            mapSegmentCorners[58, 1, 1] = -37.89;

            mapSegmentCorners[59, 0, 0] = -4.565;
            mapSegmentCorners[59, 0, 1] = -37.89;
            mapSegmentCorners[59, 1, 0] = 1.25;
            mapSegmentCorners[59, 1, 1] = -37.89;

            mapSegmentCorners[60, 0, 0] = -4.565;
            mapSegmentCorners[60, 0, 1] = -30.73;
            mapSegmentCorners[60, 1, 0] = -4.565;
            mapSegmentCorners[60, 1, 1] = -37.89;

            mapSegmentCorners[61, 0, 0] = 14.385;
            mapSegmentCorners[61, 0, 1] = -30.73;
            mapSegmentCorners[61, 1, 0] = 18.345;
            mapSegmentCorners[61, 1, 1] = -30.73;

            mapSegmentCorners[62, 0, 0] = 14.385;
            mapSegmentCorners[62, 0, 1] = -30.73;
            mapSegmentCorners[62, 1, 0] = 14.385;
            mapSegmentCorners[62, 1, 1] = -34.23;

            mapSegmentCorners[63, 0, 0] = 4.75;
            mapSegmentCorners[63, 0, 1] = -34.23;
            mapSegmentCorners[63, 1, 0] = 14.385;
            mapSegmentCorners[63, 1, 1] = -34.23;

            mapSegmentCorners[64, 0, 0] = 18.345;
            mapSegmentCorners[64, 0, 1] = -30.73;
            mapSegmentCorners[64, 1, 0] = 18.345;
            mapSegmentCorners[64, 1, 1] = -37.89;

            mapSegmentCorners[65, 0, 0] = 4.75;
            mapSegmentCorners[65, 0, 1] = -37.89;
            mapSegmentCorners[65, 1, 0] = 18.345;
            mapSegmentCorners[65, 1, 1] = -37.89;

            mapSegmentCorners[66, 0, 0] = 4.75;
            mapSegmentCorners[66, 0, 1] = -34.23;
            mapSegmentCorners[66, 1, 0] = 4.75;
            mapSegmentCorners[66, 1, 1] = -37.89;

            mapSegmentCorners[67, 0, 0] = 4.75;
            mapSegmentCorners[67, 0, 1] = -20.57;
            mapSegmentCorners[67, 1, 0] = 18.345;
            mapSegmentCorners[67, 1, 1] = -20.57;

            mapSegmentCorners[68, 0, 0] = 14.385;
            mapSegmentCorners[68, 0, 1] = -24.23;
            mapSegmentCorners[68, 1, 0] = 14.385;
            mapSegmentCorners[68, 1, 1] = -27.73;

            mapSegmentCorners[69, 0, 0] = 4.75;
            mapSegmentCorners[69, 0, 1] = -24.23;
            mapSegmentCorners[69, 1, 0] = 14.385;
            mapSegmentCorners[69, 1, 1] = -24.23;

            mapSegmentCorners[70, 0, 0] = 18.345;
            mapSegmentCorners[70, 0, 1] = -20.57;
            mapSegmentCorners[70, 1, 0] = 18.345;
            mapSegmentCorners[70, 1, 1] = -27.73;

            mapSegmentCorners[71, 0, 0] = 14.385;
            mapSegmentCorners[71, 0, 1] = -27.73;
            mapSegmentCorners[71, 1, 0] = 18.345;
            mapSegmentCorners[71, 1, 1] = -27.73;

            mapSegmentCorners[72, 0, 0] = 4.75;
            mapSegmentCorners[72, 0, 1] = -20.57;
            mapSegmentCorners[72, 1, 0] = 4.75;
            mapSegmentCorners[72, 1, 1] = -24.23;

            mapSegmentCorners[73, 0, 0] = 2.85;
            mapSegmentCorners[73, 0, 1] = -43.99;
            mapSegmentCorners[73, 1, 0] = 3.15;
            mapSegmentCorners[73, 1, 1] = -43.99;

            mapSegmentCorners[74, 0, 0] = 2.85;
            mapSegmentCorners[74, 0, 1] = -44.29;
            mapSegmentCorners[74, 1, 0] = 3.15;
            mapSegmentCorners[74, 1, 1] = -44.29;

            mapSegmentCorners[75, 0, 0] = 2.85;
            mapSegmentCorners[75, 0, 1] = -43.99;
            mapSegmentCorners[75, 1, 0] = 2.85;
            mapSegmentCorners[75, 1, 1] = -44.29;

            mapSegmentCorners[76, 0, 0] = 3.15;
            mapSegmentCorners[76, 0, 1] = -43.99;
            mapSegmentCorners[76, 1, 0] = 3.15;
            mapSegmentCorners[76, 1, 1] = -44.29;

            mapSegmentCorners[77, 0, 0] = 2.85;
            mapSegmentCorners[77, 0, 1] = -47.14;
            mapSegmentCorners[77, 1, 0] = 3.15;
            mapSegmentCorners[77, 1, 1] = -47.14;

            mapSegmentCorners[78, 0, 0] = 2.85;
            mapSegmentCorners[78, 0, 1] = -47.44;
            mapSegmentCorners[78, 1, 0] = 3.15;
            mapSegmentCorners[78, 1, 1] = -47.44;

            mapSegmentCorners[79, 0, 0] = 2.85;
            mapSegmentCorners[79, 0, 1] = -47.14;
            mapSegmentCorners[79, 1, 0] = 2.85;
            mapSegmentCorners[79, 1, 1] = -47.44;

            mapSegmentCorners[80, 0, 0] = 3.15;
            mapSegmentCorners[80, 0, 1] = -47.14;
            mapSegmentCorners[80, 1, 0] = 3.15;
            mapSegmentCorners[80, 1, 1] = -47.44;

            mapSegmentCorners[81, 0, 0] = -0.5;
            mapSegmentCorners[81, 0, 1] = -43.99;
            mapSegmentCorners[81, 1, 0] = -0.2;
            mapSegmentCorners[81, 1, 1] = -43.99;

            mapSegmentCorners[82, 0, 0] = -0.5;
            mapSegmentCorners[82, 0, 1] = -44.29;
            mapSegmentCorners[82, 1, 0] = -0.2;
            mapSegmentCorners[82, 1, 1] = -44.29;

            mapSegmentCorners[83, 0, 0] = -0.5;
            mapSegmentCorners[83, 0, 1] = -43.99;
            mapSegmentCorners[83, 1, 0] = -0.5;
            mapSegmentCorners[83, 1, 1] = -44.29;

            mapSegmentCorners[84, 0, 0] = -0.2;
            mapSegmentCorners[84, 0, 1] = -43.99;
            mapSegmentCorners[84, 1, 0] = -0.2;
            mapSegmentCorners[84, 1, 1] = -44.29;

            mapSegmentCorners[85, 0, 0] = -0.5;
            mapSegmentCorners[85, 0, 1] = -47.14;
            mapSegmentCorners[85, 1, 0] = -0.2;
            mapSegmentCorners[85, 1, 1] = -47.14;

            mapSegmentCorners[86, 0, 0] = -0.5;
            mapSegmentCorners[86, 0, 1] = -47.44;
            mapSegmentCorners[86, 1, 0] = -0.2;
            mapSegmentCorners[86, 1, 1] = -47.44;

            mapSegmentCorners[87, 0, 0] = -0.5;
            mapSegmentCorners[87, 0, 1] = -47.14;
            mapSegmentCorners[87, 1, 0] = -0.5;
            mapSegmentCorners[87, 1, 1] = -47.44;

            mapSegmentCorners[88, 0, 0] = -0.2;
            mapSegmentCorners[88, 0, 1] = -47.14;
            mapSegmentCorners[88, 1, 0] = -0.2;
            mapSegmentCorners[88, 1, 1] = -47.44;

            mapSegmentCorners[89, 0, 0] = 6.42;
            mapSegmentCorners[89, 0, 1] = -43.99;
            mapSegmentCorners[89, 1, 0] = 6.72;
            mapSegmentCorners[89, 1, 1] = -43.99;

            mapSegmentCorners[90, 0, 0] = 6.42;
            mapSegmentCorners[90, 0, 1] = -44.29;
            mapSegmentCorners[90, 1, 0] = 6.72;
            mapSegmentCorners[90, 1, 1] = -44.29;

            mapSegmentCorners[91, 0, 0] = 6.42;
            mapSegmentCorners[91, 0, 1] = -43.99;
            mapSegmentCorners[91, 1, 0] = 6.42;
            mapSegmentCorners[91, 1, 1] = -44.29;

            mapSegmentCorners[92, 0, 0] = 6.72;
            mapSegmentCorners[92, 0, 1] = -43.99;
            mapSegmentCorners[92, 1, 0] = 6.72;
            mapSegmentCorners[92, 1, 1] = -44.29;

            mapSegmentCorners[93, 0, 0] = 6.42;
            mapSegmentCorners[93, 0, 1] = -47.14;
            mapSegmentCorners[93, 1, 0] = 6.72;
            mapSegmentCorners[93, 1, 1] = -47.14;

            mapSegmentCorners[94, 0, 0] = 6.42;
            mapSegmentCorners[94, 0, 1] = -47.44;
            mapSegmentCorners[94, 1, 0] = 6.72;
            mapSegmentCorners[94, 1, 1] = -47.44;

            mapSegmentCorners[95, 0, 0] = 6.42;
            mapSegmentCorners[95, 0, 1] = -47.14;
            mapSegmentCorners[95, 1, 0] = 6.42;
            mapSegmentCorners[95, 1, 1] = -47.44;

            mapSegmentCorners[96, 0, 0] = 6.72;
            mapSegmentCorners[96, 0, 1] = -47.14;
            mapSegmentCorners[96, 1, 0] = 6.72;
            mapSegmentCorners[96, 1, 1] = -47.44;

            mapSegmentCorners[97, 0, 0] = 9.99;
            mapSegmentCorners[97, 0, 1] = -43.99;
            mapSegmentCorners[97, 1, 0] = 10.29;
            mapSegmentCorners[97, 1, 1] = -43.99;

            mapSegmentCorners[98, 0, 0] = 9.99;
            mapSegmentCorners[98, 0, 1] = -44.29;
            mapSegmentCorners[98, 1, 0] = 10.29;
            mapSegmentCorners[98, 1, 1] = -44.29;

            mapSegmentCorners[99, 0, 0] = 9.99;
            mapSegmentCorners[99, 0, 1] = -43.99;
            mapSegmentCorners[99, 1, 0] = 9.99;
            mapSegmentCorners[99, 1, 1] = -44.29;

            mapSegmentCorners[100, 0, 0] = 10.29;
            mapSegmentCorners[100, 0, 1] = -43.99;
            mapSegmentCorners[100, 1, 0] = 10.29;
            mapSegmentCorners[100, 1, 1] = -44.29;

            mapSegmentCorners[101, 0, 0] = 9.99;
            mapSegmentCorners[101, 0, 1] = -47.14;
            mapSegmentCorners[101, 1, 0] = 10.29;
            mapSegmentCorners[101, 1, 1] = -47.14;

            mapSegmentCorners[102, 0, 0] = 9.99;
            mapSegmentCorners[102, 0, 1] = -47.44;
            mapSegmentCorners[102, 1, 0] = 10.29;
            mapSegmentCorners[102, 1, 1] = -47.44;

            mapSegmentCorners[103, 0, 0] = 9.99;
            mapSegmentCorners[103, 0, 1] = -47.14;
            mapSegmentCorners[103, 1, 0] = 9.99;
            mapSegmentCorners[103, 1, 1] = -47.44;

            mapSegmentCorners[104, 0, 0] = 10.29;
            mapSegmentCorners[104, 0, 1] = -47.14;
            mapSegmentCorners[104, 1, 0] = 10.29;
            mapSegmentCorners[104, 1, 1] = -47.44;

            mapSegmentCorners[105, 0, 0] = 13.56;
            mapSegmentCorners[105, 0, 1] = -43.99;
            mapSegmentCorners[105, 1, 0] = 13.86;
            mapSegmentCorners[105, 1, 1] = -43.99;

            mapSegmentCorners[106, 0, 0] = 13.56;
            mapSegmentCorners[106, 0, 1] = -44.29;
            mapSegmentCorners[106, 1, 0] = 13.86;
            mapSegmentCorners[106, 1, 1] = -44.29;

            mapSegmentCorners[107, 0, 0] = 13.56;
            mapSegmentCorners[107, 0, 1] = -43.99;
            mapSegmentCorners[107, 1, 0] = 13.56;
            mapSegmentCorners[107, 1, 1] = -44.29;

            mapSegmentCorners[108, 0, 0] = 13.86;
            mapSegmentCorners[108, 0, 1] = -43.99;
            mapSegmentCorners[108, 1, 0] = 13.86;
            mapSegmentCorners[108, 1, 1] = -44.29;

            mapSegmentCorners[109, 0, 0] = 13.56;
            mapSegmentCorners[109, 0, 1] = -47.14;
            mapSegmentCorners[109, 1, 0] = 13.86;
            mapSegmentCorners[109, 1, 1] = -47.14;

            mapSegmentCorners[110, 0, 0] = 13.56;
            mapSegmentCorners[110, 0, 1] = -47.44;
            mapSegmentCorners[110, 1, 0] = 13.86;
            mapSegmentCorners[110, 1, 1] = -47.44;

            mapSegmentCorners[111, 0, 0] = 13.56;
            mapSegmentCorners[111, 0, 1] = -47.14;
            mapSegmentCorners[111, 1, 0] = 13.56;
            mapSegmentCorners[111, 1, 1] = -47.44;

            mapSegmentCorners[112, 0, 0] = 13.86;
            mapSegmentCorners[112, 0, 1] = -47.14;
            mapSegmentCorners[112, 1, 0] = 13.86;
            mapSegmentCorners[112, 1, 1] = -47.44;

            mapSegmentCorners[113, 0, 0] = 17.13;
            mapSegmentCorners[113, 0, 1] = -43.99;
            mapSegmentCorners[113, 1, 0] = 17.43;
            mapSegmentCorners[113, 1, 1] = -43.99;

            mapSegmentCorners[114, 0, 0] = 17.13;
            mapSegmentCorners[114, 0, 1] = -44.29;
            mapSegmentCorners[114, 1, 0] = 17.43;
            mapSegmentCorners[114, 1, 1] = -44.29;

            mapSegmentCorners[115, 0, 0] = 17.13;
            mapSegmentCorners[115, 0, 1] = -43.99;
            mapSegmentCorners[115, 1, 0] = 17.13;
            mapSegmentCorners[115, 1, 1] = -44.29;

            mapSegmentCorners[116, 0, 0] = 17.43;
            mapSegmentCorners[116, 0, 1] = -43.99;
            mapSegmentCorners[116, 1, 0] = 17.43;
            mapSegmentCorners[116, 1, 1] = -44.29;

            mapSegmentCorners[117, 0, 0] = 17.13;
            mapSegmentCorners[117, 0, 1] = -47.14;
            mapSegmentCorners[117, 1, 0] = 17.43;
            mapSegmentCorners[117, 1, 1] = -47.14;

            mapSegmentCorners[118, 0, 0] = 17.13;
            mapSegmentCorners[118, 0, 1] = -47.44;
            mapSegmentCorners[118, 1, 0] = 17.43;
            mapSegmentCorners[118, 1, 1] = -47.44;

            mapSegmentCorners[119, 0, 0] = 17.13;
            mapSegmentCorners[119, 0, 1] = -47.14;
            mapSegmentCorners[119, 1, 0] = 17.13;
            mapSegmentCorners[119, 1, 1] = -47.44;

            mapSegmentCorners[120, 0, 0] = 17.43;
            mapSegmentCorners[120, 0, 1] = -47.14;
            mapSegmentCorners[120, 1, 0] = 17.43;
            mapSegmentCorners[120, 1, 1] = -47.44;

            mapSegmentCorners[121, 0, 0] = 20.7;
            mapSegmentCorners[121, 0, 1] = -43.99;
            mapSegmentCorners[121, 1, 0] = 21.0;
            mapSegmentCorners[121, 1, 1] = -43.99;

            mapSegmentCorners[122, 0, 0] = 20.7;
            mapSegmentCorners[122, 0, 1] = -44.29;
            mapSegmentCorners[122, 1, 0] = 21.0;
            mapSegmentCorners[122, 1, 1] = -44.29;

            mapSegmentCorners[123, 0, 0] = 20.7;
            mapSegmentCorners[123, 0, 1] = -43.99;
            mapSegmentCorners[123, 1, 0] = 20.7;
            mapSegmentCorners[123, 1, 1] = -44.29;

            mapSegmentCorners[124, 0, 0] = 21.0;
            mapSegmentCorners[124, 0, 1] = -43.99;
            mapSegmentCorners[124, 1, 0] = 21.0;
            mapSegmentCorners[124, 1, 1] = -44.29;

            mapSegmentCorners[125, 0, 0] = 20.7;
            mapSegmentCorners[125, 0, 1] = -47.14;
            mapSegmentCorners[125, 1, 0] = 21.0;
            mapSegmentCorners[125, 1, 1] = -47.14;

            mapSegmentCorners[126, 0, 0] = 20.7;
            mapSegmentCorners[126, 0, 1] = -47.44;
            mapSegmentCorners[126, 1, 0] = 21.0;
            mapSegmentCorners[126, 1, 1] = -47.44;

            mapSegmentCorners[127, 0, 0] = 20.7;
            mapSegmentCorners[127, 0, 1] = -47.14;
            mapSegmentCorners[127, 1, 0] = 20.7;
            mapSegmentCorners[127, 1, 1] = -47.44;

            mapSegmentCorners[128, 0, 0] = 21.0;
            mapSegmentCorners[128, 0, 1] = -47.14;
            mapSegmentCorners[128, 1, 0] = 21.0;
            mapSegmentCorners[128, 1, 1] = -47.44;

            mapSegmentCorners[129, 0, 0] = 24.27;
            mapSegmentCorners[129, 0, 1] = -43.99;
            mapSegmentCorners[129, 1, 0] = 24.57;
            mapSegmentCorners[129, 1, 1] = -43.99;

            mapSegmentCorners[130, 0, 0] = 24.27;
            mapSegmentCorners[130, 0, 1] = -44.29;
            mapSegmentCorners[130, 1, 0] = 24.57;
            mapSegmentCorners[130, 1, 1] = -44.29;

            mapSegmentCorners[131, 0, 0] = 24.27;
            mapSegmentCorners[131, 0, 1] = -43.99;
            mapSegmentCorners[131, 1, 0] = 24.27;
            mapSegmentCorners[131, 1, 1] = -44.29;

            mapSegmentCorners[132, 0, 0] = 24.57;
            mapSegmentCorners[132, 0, 1] = -43.99;
            mapSegmentCorners[132, 1, 0] = 24.57;
            mapSegmentCorners[132, 1, 1] = -44.29;

            mapSegmentCorners[133, 0, 0] = 24.27;
            mapSegmentCorners[133, 0, 1] = -47.14;
            mapSegmentCorners[133, 1, 0] = 24.57;
            mapSegmentCorners[133, 1, 1] = -47.14;

            mapSegmentCorners[134, 0, 0] = 24.27;
            mapSegmentCorners[134, 0, 1] = -47.44;
            mapSegmentCorners[134, 1, 0] = 24.57;
            mapSegmentCorners[134, 1, 1] = -47.44;

            mapSegmentCorners[135, 0, 0] = 24.27;
            mapSegmentCorners[135, 0, 1] = -47.14;
            mapSegmentCorners[135, 1, 0] = 24.27;
            mapSegmentCorners[135, 1, 1] = -47.44;

            mapSegmentCorners[136, 0, 0] = 24.57;
            mapSegmentCorners[136, 0, 1] = -47.14;
            mapSegmentCorners[136, 1, 0] = 24.57;
            mapSegmentCorners[136, 1, 1] = -47.44;

            mapSegmentCorners[137, 0, 0] = 21.591;
            mapSegmentCorners[137, 0, 1] = -20.57;
            mapSegmentCorners[137, 1, 0] = 21.891;
            mapSegmentCorners[137, 1, 1] = -20.57;

            mapSegmentCorners[138, 0, 0] = 21.591;
            mapSegmentCorners[138, 0, 1] = -20.87;
            mapSegmentCorners[138, 1, 0] = 21.891;
            mapSegmentCorners[138, 1, 1] = -20.87;

            mapSegmentCorners[139, 0, 0] = 21.591;
            mapSegmentCorners[139, 0, 1] = -20.57;
            mapSegmentCorners[139, 1, 0] = 21.591;
            mapSegmentCorners[139, 1, 1] = -20.87;

            mapSegmentCorners[140, 0, 0] = 21.891;
            mapSegmentCorners[140, 0, 1] = -20.57;
            mapSegmentCorners[140, 1, 0] = 21.891;
            mapSegmentCorners[140, 1, 1] = -20.87;

            mapSegmentCorners[141, 0, 0] = 21.591;
            mapSegmentCorners[141, 0, 1] = -23.921;
            mapSegmentCorners[141, 1, 0] = 21.891;
            mapSegmentCorners[141, 1, 1] = -23.921;

            mapSegmentCorners[142, 0, 0] = 21.591;
            mapSegmentCorners[142, 0, 1] = -24.221;
            mapSegmentCorners[142, 1, 0] = 21.891;
            mapSegmentCorners[142, 1, 1] = -24.221;

            mapSegmentCorners[143, 0, 0] = 21.591;
            mapSegmentCorners[143, 0, 1] = -23.921;
            mapSegmentCorners[143, 1, 0] = 21.591;
            mapSegmentCorners[143, 1, 1] = -24.221;

            mapSegmentCorners[144, 0, 0] = 21.891;
            mapSegmentCorners[144, 0, 1] = -23.921;
            mapSegmentCorners[144, 1, 0] = 21.891;
            mapSegmentCorners[144, 1, 1] = -24.221;

            mapSegmentCorners[145, 0, 0] = 21.591;
            mapSegmentCorners[145, 0, 1] = -27.276;
            mapSegmentCorners[145, 1, 0] = 21.891;
            mapSegmentCorners[145, 1, 1] = -27.276;

            mapSegmentCorners[146, 0, 0] = 21.591;
            mapSegmentCorners[146, 0, 1] = -27.576;
            mapSegmentCorners[146, 1, 0] = 21.891;
            mapSegmentCorners[146, 1, 1] = -27.576;

            mapSegmentCorners[147, 0, 0] = 21.591;
            mapSegmentCorners[147, 0, 1] = -27.276;
            mapSegmentCorners[147, 1, 0] = 21.591;
            mapSegmentCorners[147, 1, 1] = -27.576;

            mapSegmentCorners[148, 0, 0] = 21.891;
            mapSegmentCorners[148, 0, 1] = -27.276;
            mapSegmentCorners[148, 1, 0] = 21.891;
            mapSegmentCorners[148, 1, 1] = -27.576;

            mapSegmentCorners[149, 0, 0] = 21.591;
            mapSegmentCorners[149, 0, 1] = -30.634;
            mapSegmentCorners[149, 1, 0] = 21.891;
            mapSegmentCorners[149, 1, 1] = -30.634;

            mapSegmentCorners[150, 0, 0] = 21.591;
            mapSegmentCorners[150, 0, 1] = -30.934;
            mapSegmentCorners[150, 1, 0] = 21.891;
            mapSegmentCorners[150, 1, 1] = -30.934;

            mapSegmentCorners[151, 0, 0] = 21.591;
            mapSegmentCorners[151, 0, 1] = -30.634;
            mapSegmentCorners[151, 1, 0] = 21.591;
            mapSegmentCorners[151, 1, 1] = -30.934;

            mapSegmentCorners[152, 0, 0] = 21.891;
            mapSegmentCorners[152, 0, 1] = -30.634;
            mapSegmentCorners[152, 1, 0] = 21.891;
            mapSegmentCorners[152, 1, 1] = -30.934;

            mapSegmentCorners[153, 0, 0] = 21.591;
            mapSegmentCorners[153, 0, 1] = -33.99;
            mapSegmentCorners[153, 1, 0] = 21.891;
            mapSegmentCorners[153, 1, 1] = -33.99;

            mapSegmentCorners[154, 0, 0] = 21.591;
            mapSegmentCorners[154, 0, 1] = -34.29;
            mapSegmentCorners[154, 1, 0] = 21.891;
            mapSegmentCorners[154, 1, 1] = -34.29;

            mapSegmentCorners[155, 0, 0] = 21.591;
            mapSegmentCorners[155, 0, 1] = -33.99;
            mapSegmentCorners[155, 1, 0] = 21.591;
            mapSegmentCorners[155, 1, 1] = -34.29;

            mapSegmentCorners[156, 0, 0] = 21.891;
            mapSegmentCorners[156, 0, 1] = -33.99;
            mapSegmentCorners[156, 1, 0] = 21.891;
            mapSegmentCorners[156, 1, 1] = -34.29;

            mapSegmentCorners[157, 0, 0] = 21.591;
            mapSegmentCorners[157, 0, 1] = -37.344;
            mapSegmentCorners[157, 1, 0] = 21.891;
            mapSegmentCorners[157, 1, 1] = -37.344;

            mapSegmentCorners[158, 0, 0] = 21.591;
            mapSegmentCorners[158, 0, 1] = -37.644;
            mapSegmentCorners[158, 1, 0] = 21.891;
            mapSegmentCorners[158, 1, 1] = -37.644;

            mapSegmentCorners[159, 0, 0] = 21.591;
            mapSegmentCorners[159, 0, 1] = -37.344;
            mapSegmentCorners[159, 1, 0] = 21.591;
            mapSegmentCorners[159, 1, 1] = -37.644;

            mapSegmentCorners[160, 0, 0] = 21.891;
            mapSegmentCorners[160, 0, 1] = -37.344;
            mapSegmentCorners[160, 1, 0] = 21.891;
            mapSegmentCorners[160, 1, 1] = -37.644;

            mapSegmentCorners[161, 0, 0] = 21.591;
            mapSegmentCorners[161, 0, 1] = -40.699;
            mapSegmentCorners[161, 1, 0] = 21.891;
            mapSegmentCorners[161, 1, 1] = -40.699;

            mapSegmentCorners[162, 0, 0] = 21.591;
            mapSegmentCorners[162, 0, 1] = -40.999;
            mapSegmentCorners[162, 1, 0] = 21.891;
            mapSegmentCorners[162, 1, 1] = -40.999;

            mapSegmentCorners[163, 0, 0] = 21.591;
            mapSegmentCorners[163, 0, 1] = -40.699;
            mapSegmentCorners[163, 1, 0] = 21.591;
            mapSegmentCorners[163, 1, 1] = -40.999;

            mapSegmentCorners[164, 0, 0] = 21.891;
            mapSegmentCorners[164, 0, 1] = -40.699;
            mapSegmentCorners[164, 1, 0] = 21.891;
            mapSegmentCorners[164, 1, 1] = -40.999;

            mapSegmentCorners[165, 0, 0] = 4.75;
            mapSegmentCorners[165, 0, 1] = -54.056;
            mapSegmentCorners[165, 1, 0] = 18.345;
            mapSegmentCorners[165, 1, 1] = -54.056;

            mapSegmentCorners[166, 0, 0] = 1.25;
            mapSegmentCorners[166, 0, 1] = -54.051;
            mapSegmentCorners[166, 1, 0] = -4.565;
            mapSegmentCorners[166, 1, 1] = -54.051;

            mapSegmentCorners[167, 0, 0] = 24.838;
            mapSegmentCorners[167, 0, 1] = -20.57;
            mapSegmentCorners[167, 1, 0] = 24.838;
            mapSegmentCorners[167, 1, 1] = -41.202;

            //*/
            /* mapSegmentCorners[0, 0, 0] = 0;
             mapSegmentCorners[0, 0, 1] = -1.13;
             mapSegmentCorners[0, 1, 0] = (8.477 - 6.41);
             mapSegmentCorners[0, 1, 1] = -1.13;

             mapSegmentCorners[1, 0, 0] = 0;
             mapSegmentCorners[1, 0, 1] = -1.13;
             mapSegmentCorners[1, 1, 0] = -1.204;
             mapSegmentCorners[1, 1, 1] = -1.13;//first wall

             mapSegmentCorners[2, 0, 0] = -1.204;
             mapSegmentCorners[2, 0, 1] = -1.13;
             mapSegmentCorners[2, 1, 0] = -1.204;
             mapSegmentCorners[2, 1, 1] = 8.452;//long wall

             mapSegmentCorners[3, 0, 0] = (8.477 - 7.469);
             mapSegmentCorners[3, 0, 1] = 1.13;
             mapSegmentCorners[3, 1, 0] = (8.477 - 2.416);
             mapSegmentCorners[3, 1, 1] = 1.13;

             mapSegmentCorners[4, 0, 0] = (8.477 - 2.416);
             mapSegmentCorners[4, 0, 1] = 1.13;
             mapSegmentCorners[4, 1, 0] = (8.477 - 2.416);
             mapSegmentCorners[4, 1, 1] = 1.13 + (8.435 - 2.284);

             mapSegmentCorners[5, 0, 0] = (8.477 - 7.469);
             mapSegmentCorners[5, 0, 1] = -(-1.13 - (8.435 - 2.284));
             mapSegmentCorners[5, 1, 0] = (8.477 - 2.416);
             mapSegmentCorners[5, 1, 1] = 1.13 + (8.435 - 2.284);

             mapSegmentCorners[6, 0, 0] = (8.477 - 7.469);
             mapSegmentCorners[6, 0, 1] = 1.13;
             mapSegmentCorners[6, 1, 0] = (8.477 - 7.469);
             mapSegmentCorners[6, 1, 1] = 1.13 + (8.435 - 2.284);
             //8.435 - 2.284//center wall

             mapSegmentCorners[7, 0, 0] = (8.477 - 6.41);
             mapSegmentCorners[7, 0, 1] = -1.13;
             mapSegmentCorners[7, 1, 0] = (8.477 - 6.41);
             mapSegmentCorners[7, 1, 1] = -(2.2 + 1.13);

             mapSegmentCorners[8, 0, 0] = (8.477 - 6.41);
             mapSegmentCorners[8, 0, 1] = -(2.2 + 1.13);
             mapSegmentCorners[8, 1, 0] = (8.477 - 6.41 - (9.421 - 3.575));
             mapSegmentCorners[8, 1, 1] = -(2.2 + 1.13);//opening to sprague;


             mapSegmentCorners[9, 0, 0] = (8.477 - 6.41 + 3.506);
             mapSegmentCorners[9, 0, 1] = -1.13;
             mapSegmentCorners[9, 1, 0] = (8.477 - 6.41 + 3.506);
             mapSegmentCorners[9, 1, 1] = -(2.2 + 1.13);

             mapSegmentCorners[11, 0, 0] = (8.477 - 6.41 + 3.506);
             mapSegmentCorners[11, 0, 1] = -(2.2 + 1.13);
             mapSegmentCorners[11, 1, 0] = (8.477 - 6.41 + 3.506 + (9.421 - 3.575));
             mapSegmentCorners[11, 1, 1] = -(2.2 + 1.13);

             mapSegmentCorners[10, 0, 0] = (8.477 - 6.41 + 3.506);
             mapSegmentCorners[10, 0, 1] = -1.13;
             mapSegmentCorners[10, 1, 0] = (8.477 - 0.07);
             mapSegmentCorners[10, 1, 1] = -1.13;

             mapSegmentCorners[12, 0, 0] = (8.477 - 0.07);
             mapSegmentCorners[12, 0, 1] = -1.13;
             mapSegmentCorners[12, 1, 0] = (8.477 - 0.07);
             mapSegmentCorners[12, 1, 1] = 8.452;
             //other side

             mapSegmentCorners[13, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
             mapSegmentCorners[13, 0, 1] = -(2.2 + 1.13 + 6.984);
             mapSegmentCorners[13, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
             mapSegmentCorners[13, 1, 1] = -(2.2 + 1.13 + 6.984);

             mapSegmentCorners[14, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
             mapSegmentCorners[14, 0, 1] = -(2.2 + 1.13 + 6.984);
             mapSegmentCorners[14, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
             mapSegmentCorners[14, 1, 1] = -(2.2+ 1.13 + 6.984);//pillars

             mapSegmentCorners[15, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
             mapSegmentCorners[15, 0, 1] = -(2.2 + 1.13 + 6.984);
             mapSegmentCorners[15, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
             mapSegmentCorners[15, 1, 1] = -(2.2 + 1.13 + 6.984);//pillars

             mapSegmentCorners[16, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
             mapSegmentCorners[16, 0, 1] = -(2.2 + 1.13 + 6.984);
             mapSegmentCorners[16, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
             mapSegmentCorners[16, 1, 1] = -(2.2 + 1.13 + 6.984);//pillars

             mapSegmentCorners[17, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
             mapSegmentCorners[17, 0, 1] = -(2.2 + 1.13 + 6.984);
             mapSegmentCorners[17, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
             mapSegmentCorners[17, 1, 1] = -(2.2 + 1.13 + 6.984);//pillars

             ///////
             mapSegmentCorners[30, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
             mapSegmentCorners[30, 0, 1] = -(2.2 + 1.13 + 6.984+(6.8 - 6.498));
             mapSegmentCorners[30, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
             mapSegmentCorners[30, 1, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));

             mapSegmentCorners[31, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
             mapSegmentCorners[31, 0, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));
             mapSegmentCorners[31, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
             mapSegmentCorners[31, 1, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));//pillars

             mapSegmentCorners[32, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
             mapSegmentCorners[32, 0, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));
             mapSegmentCorners[32, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
             mapSegmentCorners[32, 1, 1] = -(2.2 + 1.13 + 6.984+(6.8 - 6.498));//pillars

             mapSegmentCorners[33, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
             mapSegmentCorners[33, 0, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));
             mapSegmentCorners[33, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
             mapSegmentCorners[33, 1, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));//pillars

             mapSegmentCorners[34, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
             mapSegmentCorners[34, 0, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));
             mapSegmentCorners[34, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
             mapSegmentCorners[34, 1, 1] = -(2.2 + 1.13 + 6.984 + (6.8 - 6.498));//pillars
             ////other side

             mapSegmentCorners[18, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
             mapSegmentCorners[18, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162);
             mapSegmentCorners[18, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
             mapSegmentCorners[18, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162);

             mapSegmentCorners[23, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
             mapSegmentCorners[23, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162+(6.8-6.498));
             mapSegmentCorners[23, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
             mapSegmentCorners[23, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162+(6.8 - 6.498));

             mapSegmentCorners[19, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
             mapSegmentCorners[19, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162);
             mapSegmentCorners[19, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
             mapSegmentCorners[19, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162);//pillars

             mapSegmentCorners[20, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
             mapSegmentCorners[20, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162);
             mapSegmentCorners[20, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
             mapSegmentCorners[20, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162);//pillars

             mapSegmentCorners[21, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
             mapSegmentCorners[21, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162);
             mapSegmentCorners[21, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
             mapSegmentCorners[21, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162);//pillars

             mapSegmentCorners[22, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
             mapSegmentCorners[22, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162);
             mapSegmentCorners[22, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
             mapSegmentCorners[22, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162);//pillars

             //pillar with width
             mapSegmentCorners[24, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
             mapSegmentCorners[24, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
             mapSegmentCorners[24, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
             mapSegmentCorners[24, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars

             mapSegmentCorners[25, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
             mapSegmentCorners[25, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
             mapSegmentCorners[25, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
             mapSegmentCorners[25, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars

             mapSegmentCorners[26, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
             mapSegmentCorners[26, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
             mapSegmentCorners[26, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
             mapSegmentCorners[26, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars

             mapSegmentCorners[27, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
             mapSegmentCorners[27, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
             mapSegmentCorners[27, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
             mapSegmentCorners[27, 1, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars


             /////////////////////////////////////
             mapSegmentCorners[28, 0, 0] = (8.477 - 6.41 +(9.401-9.317));
             mapSegmentCorners[28, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498)+6.425);
             mapSegmentCorners[28, 1, 0] = (8.477 - 6.41 + (9.401 - 9.317)-(8.478-2.964));
             mapSegmentCorners[28, 1, 1] = -(2.2 + 1.13 + 17.163);//other Wall

             mapSegmentCorners[29, 0, 0] = (8.477 - 6.41 + (9.401 - 9.317) + 3.518);
             mapSegmentCorners[29, 0, 1] = -(2.2 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498) + 6.425);
             mapSegmentCorners[29, 1, 0] = (8.477 - 6.41 + (9.401 - 9.317) + 3.518 + (20.025-6.936));
             mapSegmentCorners[29, 1, 1] = -(2.2 + 1.13 + 17.163);//other Wall
             //vertical walls
             //*/
            /////////////////////////////////////////////////////////////s
            /* numMapSegments = 8;
             mapSegmentCorners = new double[numMapSegments, 2, 2];
             slopes = new double[numMapSegments];
             intercepts = new double[numMapSegments];
             segmentSizes = new double[numMapSegments];

             mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
             mapSegmentCorners[0, 0, 1] = 2.794;
             mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
             mapSegmentCorners[0, 1, 1] = 2.794;

             mapSegmentCorners[1, 0, 0] = -3.55 / 2;
             mapSegmentCorners[1, 0, 1] = 0.0;
             mapSegmentCorners[1, 1, 0] = -3.55 / 2;
             mapSegmentCorners[1, 1, 1] = -2.74;

             mapSegmentCorners[2, 0, 0] = 3.55 / 2;
             mapSegmentCorners[2, 0, 1] = 0.0;
             mapSegmentCorners[2, 1, 0] = 3.55 / 2;
             mapSegmentCorners[2, 1, 1] = -2.74;

             mapSegmentCorners[3, 0, 0] = 3.55 / 2;
             mapSegmentCorners[3, 0, 1] = 0.0;
             mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
             mapSegmentCorners[3, 1, 1] = 0.0;

             mapSegmentCorners[4, 0, 0] = -3.55 / 2;
             mapSegmentCorners[4, 0, 1] = 0.0;
             mapSegmentCorners[4, 1, 0] = -3.55 / 2 - 5.79;
             mapSegmentCorners[4, 1, 1] = 0.0;

             mapSegmentCorners[5, 0, 0] = -3.55 / 2;
             mapSegmentCorners[5, 0, 1] = -2.74;
             mapSegmentCorners[5, 1, 0] = -3.55 / 2 - 3.05;
             mapSegmentCorners[5, 1, 1] = -2.74;

             mapSegmentCorners[6, 0, 0] = 3.55 / 2;
             mapSegmentCorners[6, 0, 1] = -2.74;
             mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
             mapSegmentCorners[6, 1, 1] = -2.74;

             mapSegmentCorners[7, 0, 0] = 5.03 / 2;
             mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
             mapSegmentCorners[7, 1, 0] = -5.03 / 2;
             mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;*/

            // ****************** Additional Student Code: End   ************
            #endregion
            // Set map parameters
            // These will be useful in your future coding.
            minX = 9999; minY = 9999; maxX = -9999; maxY = -9999;
            for (int i = 0; i < numMapSegments; i++)
            {

                // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i, 0, 0], mapSegmentCorners[i, 1, 0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i, 0, 1], mapSegmentCorners[i, 1, 1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i, 0, 0], mapSegmentCorners[i, 1, 0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i, 0, 1], mapSegmentCorners[i, 1, 1]));

                // Set wall segments to be horizontal
                slopes[i] = (mapSegmentCorners[i, 0, 1] - mapSegmentCorners[i, 1, 1]) / (0.001 + mapSegmentCorners[i, 0, 0] - mapSegmentCorners[i, 1, 0]);
                intercepts[i] = mapSegmentCorners[i, 0, 1] - slopes[i] * mapSegmentCorners[i, 0, 0];

                // Set wall segment lengths
                segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i, 0, 0] - mapSegmentCorners[i, 1, 0], 2) + Math.Pow(mapSegmentCorners[i, 0, 1] - mapSegmentCorners[i, 1, 1], 2));
            }

            //hardcode the throw zones
            numNThrowZ = 17;
            /* numThrowZ = 4;
             Zone z1 = new Zone(-10.945, 0, 0, 2.74);
             Zone z2 = new Zone( 0, 10.945, 0, 2.74);
             Zone z3 = new Zone(-1.775, 1.775, -2.74, 0);
             Zone z4 = new Zone(-4.825, 4.825, -5.05, -2.74);
             mapthrowZ = new List<Zone>();
             mapthrowZ.Add(z1);
             mapthrowZ.Add(z2);
             mapthrowZ.Add(z3);
             mapthrowZ.Add(z4);
             //*/
            mapNthrowZ = new List<Zone>();
            mapNthrowZ.Add(new Zone(13.794, -7.945, double.PositiveInfinity, 13.794));
            mapNthrowZ.Add(new Zone(0.485, 5.515, 5.95, 0.93));

            mapNthrowZ.Add(new Zone(double.PositiveInfinity, 13.945, 13.794, -7.9));

            mapNthrowZ.Add(new Zone(-7.945, double.NegativeInfinity, 13.794, -7.9));

            mapNthrowZ.Add(new Zone(-1.825, -4.565, 8.26, -1.32));

            mapNthrowZ.Add(new Zone(13.945, 7.825, 8.26, -1.32));

            mapNthrowZ.Add(new Zone(-11.32, 2.85, -11.02, 3.15));

            mapNthrowZ.Add(new Zone(-14.47, 2.85, -14.17, 3.15));

            mapNthrowZ.Add(new Zone(-20.57, 1.25, -20.57, 18.345));

            mapNthrowZ.Add(new Zone(-7.235, double.NegativeInfinity, -20.57, -38.81));

            mapNthrowZ.Add(new Zone(-0.57, -4.565, -20.57, -27.73));

            mapNthrowZ.Add(new Zone(1.25, -0.57, -20.57, -24.23));

            mapNthrowZ.Add(new Zone(-0.57, -4.565, -30.73, -34.23));

            mapNthrowZ.Add(new Zone(1.25, -0.57, -34.23, -37.89));

            mapNthrowZ.Add(new Zone(-0.57, -4.565, -30.73, -34.23));

            mapNthrowZ.Add(new Zone(18.345, 4.75, -34.23, -37.89));

            mapNthrowZ.Add(new Zone(18.345, 4.75, -30.73, -34.23));

            mapNthrowZ.Add(new Zone(18.345, 14.385, -34.23, -37.89));

        }



        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)

        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        /*public double GetClosestWallDistance(double x, double y, double t)
        {

            double minDist = 36.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.
            for (int i = 0; i < numMapSegments; i++)
                minDist = Math.Min(minDist, GetWallDistance(x, y, t, i));


            // ****************** Additional Student Code: End   ************

            return minDist;
        }*/
        public double GetClosestWallDistance(double x, double y, double t)
        {
            double maxDist = 6.0;
            double minDist = maxDist;
            double dist = 0;
            int seg = 0;
            // ****************** Additional Student Code: Start ************

            for (int i = 0; i < numMapSegments; i++)
            {
                if (!WallClose(x, y, i))
                {
                    debugTest();
                    continue;
                }

                dist = GetWallDistance(x, y, t, i);
                if (dist > 0)
                {
                    minDist = Math.Min(minDist, dist);
                    if (minDist == dist)
                        seg = i;
                }
            }

            // ****************** Additional Student Code: End   ************

            if (minDist == maxDist)
                return 0;

            return minDist;
        }
        // Checks if a map in the wall is close enough to the robot to consider for GetClosestWall. 
        // Returns True if the wall is "close" as defined by thres, false otherwise.
        private bool WallClose(double botX, double botY, int segment)
        {
            double thres = 6;

            double dx1 = botX - mapSegmentCorners[segment, 0, 0];
            double dy1 = botY - mapSegmentCorners[segment, 0, 1];
            double dx2 = botX - mapSegmentCorners[segment, 1, 0];
            double dy2 = botY - mapSegmentCorners[segment, 1, 1];

            if (dx1 * dx1 + dy1 * dy1 < thres * thres) return true;
            if (dx2 * dx2 + dy2 * dy2 < thres * thres) return true;
            return false;
        }


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        public bool CollisionFound(Navigation.Node n1, Navigation.Node n2, double tol)
        {


            // Check that within boundaries
            if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
                return true;


            // Check for collision with walls
            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x);
            double edgeSize = Math.Sqrt(Math.Pow(n2.y - n1.y, 2) + Math.Pow(n2.x - n1.x, 2));
            double sinTheta = Math.Sin(theta);
            double cosTheta = Math.Cos(theta);

            // Loop through segments
            for (int segment = 0; segment < numMapSegments; segment++)
            {

                double distTravelledOnEdge = 0;
                double ex = n1.x, ey = n1.y;
                double distToSegment;
                while (distTravelledOnEdge - tol < edgeSize)
                {
                    distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
                    if (distToSegment - tol < 0.05)
                        return true;
                    ex += cosTheta * distToSegment;
                    ey += sinTheta * distToSegment;
                    distTravelledOnEdge += distToSegment;
                }

            }
            return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.
        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y)
        {
            double dist = 0;

            return dist;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, double t, int segment)
        {

            double wallDist;

            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];


            double intersectX = (intercepts[segment] + Math.Tan(t) * x - y) / (Math.Tan(t) - slopes[segment]);
            double intersectY = slopes[segment] * intersectX + intercepts[segment];


            bool exists = inRange(intersectX, X1, X2) && inRange(intersectY, Y1, Y2) && inFront(intersectX - x, intersectY - y, t);
            if (exists)
                wallDist = Math.Sqrt(Math.Pow(x - intersectX, 2) + Math.Pow(y - intersectY, 2));
            else
                wallDist = 0;
            // ****************** Additional Student Code: End   ************

            return wallDist;
        }


        /*********************************************************************************************************
         * 
         *                          HELPER FUNCTIONS
         * ***********************************************************************************************************
         * 
         */

        private bool inRange(double a, double one, double two)
        {
            return (a <= Math.Max(one, two) && a >= Math.Min(one, two));
        }

        private bool inFront(double dx, double dy, double t)
        {
            return (Math.Abs(Math.Atan2(dy, dx) - t) < 0.01);
        }

        public bool inMapArea(double xloc, double yloc)
        {
            for (int i = 0; i < numNThrowZ; i++)
            {
                double boundxMin = mapNthrowZ[i].minx();
                double boundxMax = mapNthrowZ[i].maxx();
                double boundyMin = mapNthrowZ[i].miny();
                double boundyMax = mapNthrowZ[i].maxy();

                if (!inRange(xloc, boundxMin, boundxMax) && !inRange(yloc, boundyMax, boundyMin))
                    return true;
            }
            return false;
        }

        public void debugTest()
        {
            return;
        }

    }
}
