﻿using AgOpenGPS;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Compression;
using System.Linq;
using System.Windows.Forms;
using System.Xml.Linq;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.ToolTip;

namespace AgOpenGPS
{
    public class CYouTurn
    {
        #region Fields
        //copy of the mainform address
        private readonly FormGPS mf;

        /// <summary>/// triggered right after youTurnTriggerPoint is set /// </summary>
        public bool isYouTurnTriggered;

        /// <summary>  /// turning right or left?/// </summary>
        public bool isYouTurnRight;
        private int semiCircleIndex = -1;

        /// <summary> /// Is the youturn button enabled? /// </summary>
        public bool isYouTurnBtnOn;

        public double boundaryAngleOffPerpendicular, youTurnRadius;

        public int rowSkipsWidth = 1, uTurnSmoothing;

        public bool alternateSkips = false, previousBigSkip = true;
        public int rowSkipsWidth2 = 3, turnSkips = 2;

        /// <summary>  /// distance from headland as offset where to start turn shape /// </summary>
        public int youTurnStartOffset;

        //guidance values
        public double distanceFromCurrentLine, uturnDistanceFromBoundary, dxAB, dyAB;

        public double distanceFromCurrentLineSteer, distanceFromCurrentLinePivot;
        public double steerAngleGu, rEastSteer, rNorthSteer, rEastPivot, rNorthPivot;
        public double pivotCurvatureOffset, lastCurveDistance = 10000;

        private int A, B;
        private bool isHeadingSameWay = true;
        public bool isTurnCreationTooClose = false, isTurnCreationNotCrossingError = false, turnTooCloseTrigger = false;

        //pure pursuit values
        public vec3 pivot = new vec3(0, 0, 0);

        public vec2 goalPointYT = new vec2(0, 0);
        public vec2 radiusPointYT = new vec2(0, 0);
        public double steerAngleYT, rEastYT, rNorthYT, ppRadiusYT;

        //list of points for scaled and rotated YouTurn line, used for pattern, dubins, abcurve, abline
        public List<vec3> ytList = new List<vec3>();
        private List<vec3> ytList2 = new List<vec3>();

        //next curve or line to build out turn and point over
        public CABCurve nextCurve;
        public CABLine nextLine;
        public vec3 nextLookPos = new vec3(0, 0, 0);

        //if we continue on the same line or change to the next one after the uTurn
        bool isOutSameCurve;

        //for 3Pt turns - second turn
        public List<vec3> pt3ListSecondLine = new List<vec3>();

        public int uTurnStyle = 0;

        public int pt3Phase = 0;
        public vec3 pt3TurnNewAB = new vec3(0, 0, 0);
        public bool isLastFrameForward = true;

        //is UTurn pattern in or out of bounds
        public bool isOutOfBounds = false;

        //sequence of operations of finding the next turn 0 to 3
        public int youTurnPhase;

        public double crossingheading = 0;

        // Returns 1 if the lines intersect, otherwis
        public double iE = 0, iN = 0;

        // the list of possible bounds points
        public List<CClose> turnClosestList = new List<CClose>();

        //point at the farthest turn segment from pivotAxle
        public CClose closestTurnPt = new CClose();

        //where the in and out tangents cross for Albin curve
        public CClose inClosestTurnPt = new CClose();
        public CClose outClosestTurnPt = new CClose();
        public CClose startOfTurnPt = new CClose();

        //how far should the distance between points on the uTurn be
        double pointSpacing;
        #endregion

        //constructor
        public CYouTurn(FormGPS _f)
        {
            mf = _f;

            uturnDistanceFromBoundary = Properties.Settings.Default.set_youTurnDistanceFromBoundary;

            //how far before or after boundary line should turn happen
            youTurnStartOffset = Properties.Settings.Default.set_youTurnExtensionLength;

            rowSkipsWidth = Properties.Settings.Default.set_youSkipWidth;
            Set_Alternate_skips();

            ytList.Capacity = 128;

            youTurnRadius = Properties.Settings.Default.set_youTurnRadius;

            uTurnStyle = Properties.Settings.Default.set_uTurnStyle;

            uTurnSmoothing = Properties.Settings.Default.setAS_uTurnSmoothing;
        }

        //Finds the point where an AB Curve crosses the turn line
        public bool BuildCurveDubinsYouTurn(bool isTurnLeft, vec3 pivotPos)
        {
            //TODO: is calculated many taimes after the priveous turn is complete
            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.width - mf.tool.overlap) * rowSkipsWidth + (isYouTurnRight ? -mf.tool.offset * 2.0 : mf.tool.offset * 2.0); //AAA change isYouTurnRight to isTurnLeft
            pointSpacing = youTurnRadius * 0.1;

            //Albin turn
            if (turnOffset > (youTurnRadius * 2.0))
            {
                return CreateCurveWideTurn(isTurnLeft, pivotPos);
            }

            //Ohmega turn
            else
            {
                return CreateCurveOmegaTurn(isTurnLeft, pivotPos);
            }
        }

        public bool BuildABLineDubinsYouTurn(bool isTurnLeft)
        {
            if (!mf.isBtnAutoSteerOn) mf.ABLine.isHeadingSameWay
                    = Math.PI - Math.Abs(Math.Abs(mf.fixHeading - mf.ABLine.abHeading) - Math.PI) < glm.PIBy2;

            double turnOffset = (mf.tool.width - mf.tool.overlap) * rowSkipsWidth
                + (isYouTurnRight ? -mf.tool.offset * 2.0 : mf.tool.offset * 2.0);

            pointSpacing = youTurnRadius * 0.1;

            //Wide turn
            if (turnOffset > (youTurnRadius * 2.0))
            {
                //return (CreateABLineOmegaTurn(isTurnLeft));
            }

            //Small turn
            else
            {
                if (uTurnStyle == 0)
                {
                    //return (CreateABLineWideTurn(isTurnLeft));
                }
                else if (uTurnStyle == 1)
                {
                   // return (KStyleTurn(isTurnLeft));
                }
            }

            return false;
        }

        private bool CreateCurveOmegaTurn(bool isTurnLeft, vec3 pivotPos)
        {
            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.width - mf.tool.overlap) * rowSkipsWidth + (isYouTurnRight ? -mf.tool.offset * 2.0 : mf.tool.offset * 2.0);

            switch (youTurnPhase)
            {
                case 0: //find the crossing points
                    if (!FindCurveTurnPoint(mf.curve))
                    {
                        FailCreate();
                        return false;
                    }

                    //save a copy 
                    inClosestTurnPt = new CClose(closestTurnPt);

                    ytList?.Clear();

                    int count = mf.curve.isHeadingSameWay ? -1 : 1;
                    int curveIndex = inClosestTurnPt.curveIndex;

                    isOutOfBounds = true;
                    int stopIfWayOut = 0;

                    double head = 0; //gives error since isnt updated

                    while (isOutOfBounds)
                    {
                        stopIfWayOut++;
                        isOutOfBounds = false;

                        //creates half a circle starting at the crossing point
                        ytList.Clear();
                        if (curveIndex >= mf.curve.curList.Count || curveIndex < 0)
                        {
                            FailCreate();
                            return false;
                        }
                        vec3 currentPos = new vec3(mf.curve.curList[curveIndex]);

                        curveIndex += count;

                        if (!mf.curve.isHeadingSameWay) currentPos.heading += Math.PI;
                        if (currentPos.heading >= glm.twoPI) currentPos.heading -= glm.twoPI;
                        head = currentPos.heading;

                        CDubins dubYouTurnPath = new CDubins();
                        CDubins.turningRadius = youTurnRadius;

                        //now we go the other way to turn round
                        double invertHead = currentPos.heading - Math.PI;
                        if (invertHead <= -Math.PI) invertHead += glm.twoPI;
                        if (invertHead >= Math.PI) invertHead -= glm.twoPI;

                        vec3 goal = new vec3();

                        //neat trick to not have to add pi/2
                        if (isTurnLeft)
                        {
                            goal.easting = mf.curve.curList[curveIndex - count].easting + (Math.Cos(-invertHead) * turnOffset);
                            goal.northing = mf.curve.curList[curveIndex - count].northing + (Math.Sin(-invertHead) * turnOffset);
                        }
                        else
                        {
                            goal.easting = mf.curve.curList[curveIndex - count].easting - (Math.Cos(-invertHead) * turnOffset);
                            goal.northing = mf.curve.curList[curveIndex - count].northing - (Math.Sin(-invertHead) * turnOffset);
                        }

                        goal.heading = invertHead;

                        //generate the turn points
                        ytList = dubYouTurnPath.GenerateDubins(currentPos, goal);
                        if (ytList.Count == 0)
                        {
                            FailCreate();
                            return false;
                        }

                        if (stopIfWayOut == 300 || curveIndex < 1 || curveIndex > (mf.curve.curList.Count - 2))
                        {
                            //for some reason it doesn't go inside boundary
                            FailCreate();
                            return false;
                        }

                        for (int i = 0; i < ytList.Count; i++)
                        {
                            if (mf.bnd.IsPointInsideTurnArea(ytList[i]) == -1)
                            {
                                isOutOfBounds = true;
                                break;
                            }
                        }
                    }

                    //too many points from Dubins - so cut
                    double distance;
                    int cnt = ytList.Count;
                    for (int i = 1; i < cnt - 2; i++)
                    {
                        distance = glm.DistanceSquared(ytList[i], ytList[i + 1]);
                        if (distance < pointSpacing)
                        {
                            ytList.RemoveAt(i + 1);
                            i--;
                            cnt = ytList.Count;
                        }
                    }

                    //move the turn to exact at the turnline
                    ytList = MoveTurnInsideTurnLine(ytList, head, false, false);
                    if (ytList.Count == 0)
                    {
                        FailCreate();
                        return false;
                    }

                    youTurnPhase = 1;

                    break;


                case 1:
                    //TODO: is this following the curve line?
                    AddSequenceLines(ytList[0].heading - Math.PI);
                    youTurnPhase = 10;
                    break;
            }
            return true;
        }

        private bool CreateCurveWideTurn(bool isTurnLeft, vec3 pivotPos)
        {
            //we are doing a wide turn
            double head = 0;
            int count = mf.curve.isHeadingSameWay ? -1 : 1;
            switch (youTurnPhase)
            {
                case 0:
                    //Create first semicircle

                    if (!FindCurveTurnPoint(mf.curve))
                    {
                        //error
                        FailCreate();
                        return false;
                    }
                    inClosestTurnPt = new CClose(closestTurnPt);
                    startOfTurnPt = new CClose(closestTurnPt);

                    int stopIfWayOut = 0;
                    isOutOfBounds = true;

                    while (isOutOfBounds)
                    {
                        isOutOfBounds = false;
                        stopIfWayOut++;

                        vec3 currentPos = new vec3(mf.curve.curList[startOfTurnPt.curveIndex]);

                        head = currentPos.heading;
                        if (!mf.curve.isHeadingSameWay) head += Math.PI;
                        if (head > glm.twoPI) head -= glm.twoPI;
                        currentPos.heading = head;

                        // creates half a circle starting at the crossing point
                        ytList.Clear();
                        ytList.Add(currentPos);

                        //Taken from Dubbins
                        while (Math.Abs(head - currentPos.heading) < Math.PI)
                        {
                            //Update the position of the car
                            currentPos.easting += pointSpacing * Math.Sin(currentPos.heading);
                            currentPos.northing += pointSpacing * Math.Cos(currentPos.heading);

                            //Which way are we turning?
                            double turnParameter = isTurnLeft ? -1.0 : 1.0;

                            //Update the heading
                            currentPos.heading += (pointSpacing / youTurnRadius) * turnParameter;

                            //Add the new coordinate to the path
                            ytList.Add(currentPos);
                        }

                        int cnt4 = ytList.Count;
                        if (cnt4 == 0)
                        {
                            FailCreate();
                            return false;
                        }

                        //Are we out of bounds?
                        for (int j = 0; j < cnt4; j += 2)
                        {
                            if (mf.bnd.IsPointInsideTurnArea(ytList[j]) != 0)
                            {
                                isOutOfBounds = true;
                                break;
                            }
                        }

                        //first check if not out of bounds, add a bit more to clear turn line, set to phase 2
                        if (!isOutOfBounds)
                        {
                            ytList = MoveTurnInsideTurnLine(ytList, head, true, false);
                            if (ytList.Count == 0)
                            {
                                FailCreate();
                                return false;
                            }
                            youTurnPhase = 1;
                            return true;
                        }

                        if (stopIfWayOut == 300 || startOfTurnPt.curveIndex < 1 || startOfTurnPt.curveIndex > (mf.curve.curList.Count - 2))
                        {
                            //for some reason it doesn't go inside boundary
                            FailCreate();
                            return false;
                        }

                        //keep moving infield till pattern is all inside
                        startOfTurnPt.curveIndex = startOfTurnPt.curveIndex + count;
                        startOfTurnPt.closePt = new vec3(mf.curve.curList[startOfTurnPt.curveIndex]);


                        //set the flag to Critical stop machine
                        if (glm.Distance(ytList[0], mf.pivotAxlePos) < 3)
                        {
                            FailCreate();
                            return false;
                        }
                    }

                    return false;

                case 1:
                    //Takes the heading of the curve to create an imaginary point on the next line
                    double headCurve = mf.curve.curList[mf.curve.currentLocationIndex].heading;
                    if (!mf.curve.isHeadingSameWay) headCurve += Math.PI;
                    if (headCurve > glm.twoPI) headCurve -= glm.twoPI;

                    double turnOffset = (mf.tool.width - mf.tool.overlap) * rowSkipsWidth + (isYouTurnRight ? -mf.tool.offset * 2.0 : mf.tool.offset * 2.0); //change isYouTurnRight?
                    vec2 tempguidanceLookPos = new vec2(mf.guidanceLookPos.easting, mf.guidanceLookPos.northing);

                    if (!isTurnLeft)
                    {
                        //creates an imaginary curveline to the right
                        mf.guidanceLookPos.easting += (Math.Cos(headCurve) * turnOffset);
                        mf.guidanceLookPos.northing -= (Math.Sin(headCurve) * turnOffset);
                    }
                    else
                    {
                        mf.guidanceLookPos.easting -= (Math.Cos(headCurve) * turnOffset);
                        mf.guidanceLookPos.northing += (Math.Sin(headCurve) * turnOffset);
                    }

                    //create the next line with this imaginary point
                    nextCurve = new CABCurve(mf);
                    nextCurve.BuildCurveCurrentList(mf.pivotAxlePos);
                    mf.guidanceLookPos = tempguidanceLookPos;
                    isOutSameCurve = false;


                    //going with or against boundary?
                    bool isTurnLineSameWay = true;
                    double headingDifference = Math.Abs(inClosestTurnPt.turnLineHeading - ytList[ytList.Count - 1].heading);
                    if (headingDifference > glm.PIBy2 && headingDifference < 3 * glm.PIBy2) isTurnLineSameWay = false;

                    if (!FindCurveOutTurnPoint(mf.curve, ref nextCurve, inClosestTurnPt, isTurnLineSameWay))
                    {
                        //error
                        FailCreate();
                        return false;
                    }
                    outClosestTurnPt = new CClose(closestTurnPt);

                    //move the turn inside of turnline with help from the crossingCurvePoints
                    isOutOfBounds = true;
                    while (isOutOfBounds)
                    {
                        isOutOfBounds = false;
                        vec3 currentPos = new vec3(nextCurve.curList[outClosestTurnPt.curveIndex]);

                        head = currentPos.heading;
                        if ((!mf.curve.isHeadingSameWay && !isOutSameCurve) || (mf.curve.isHeadingSameWay && isOutSameCurve)) head += Math.PI;
                        if (head > glm.twoPI) head -= glm.twoPI;
                        currentPos.heading = head;

                        // creates half a circle starting at the crossing point
                        ytList2?.Clear();
                        ytList2.Add(currentPos);

                        //Taken from Dubbins
                        while (Math.Abs(head - currentPos.heading) < Math.PI)
                        {
                            //Update the position of the car
                            currentPos.easting += pointSpacing * Math.Sin(currentPos.heading);
                            currentPos.northing += pointSpacing * Math.Cos(currentPos.heading);

                            //Which way are we turning?
                            double turnParameter = isTurnLeft ? 1.0 : -1.0;

                            //Update the heading
                            currentPos.heading += (pointSpacing / youTurnRadius) * turnParameter;

                            //Add the new coordinate to the path
                            ytList2.Add(currentPos);
                        }

                        int cnt3 = ytList2.Count;
                        if (cnt3 == 0)
                        {
                            FailCreate();
                            return false;
                        }

                        //Are we out of bounds?
                        for (int j = 0; j < cnt3; j += 2)
                        {
                            if (mf.bnd.IsPointInsideTurnArea(ytList2[j]) != 0)
                            {
                                isOutOfBounds = true;
                                break;
                            }
                        }

                        //first check if not out of bounds, add a bit more to clear turn line, set to phase 2
                        if (!isOutOfBounds)
                        {
                            ytList2 = MoveTurnInsideTurnLine(ytList2, head, true, true);
                            if (ytList2.Count == 0)
                            {
                                FailCreate();
                                return false;
                            }
                            youTurnPhase = 2;
                            return true;
                        }

                        if (outClosestTurnPt.curveIndex < 1 || outClosestTurnPt.curveIndex > (nextCurve.curList.Count - 2))
                        {
                            //for some reason it doesn't go inside boundary
                            FailCreate();
                            return false;
                        }

                        //keep moving infield till pattern is all inside
                        if(!isOutSameCurve) outClosestTurnPt.curveIndex = outClosestTurnPt.curveIndex + count;
                        else outClosestTurnPt.curveIndex = outClosestTurnPt.curveIndex - count;
                        outClosestTurnPt.closePt = new vec3(nextCurve.curList[outClosestTurnPt.curveIndex]);

                    }
                    return false;

                case 2:
                    //Bind the two turns together
                    int cnt1 = ytList.Count;
                    int cnt2 = ytList2.Count;

                    //Find if the turn goes same way as turnline heading
                    bool isFirstTurnLineSameWay = true;
                    double firstHeadingDifference = Math.Abs(inClosestTurnPt.turnLineHeading - ytList[ytList.Count - 1].heading);
                    if (firstHeadingDifference > glm.PIBy2 && firstHeadingDifference < 3 * glm.PIBy2) isFirstTurnLineSameWay = false;

                    //finds out start and goal point along the tunline
                    FindInnerTurnPoints(ytList[cnt1 - 1], ytList[0].heading, inClosestTurnPt, isFirstTurnLineSameWay);
                    CClose startClosestTurnPt = new CClose(closestTurnPt);

                    FindInnerTurnPoints(ytList2[cnt2 - 1], ytList2[0].heading + Math.PI, outClosestTurnPt, !isFirstTurnLineSameWay);
                    CClose goalClosestTurnPt = new CClose(closestTurnPt);

                    //we have 2 different turnLine crossings
                    if (startClosestTurnPt.turnLineNum != goalClosestTurnPt.turnLineNum)
                    {
                        FailCreate();
                        return false;
                    }

                    //segment index is the "A" of the segment. segmentIndex+1 would be the "B"
                    //is in and out on same segment? so only 1 segment
                    if (startClosestTurnPt.turnLineIndex == goalClosestTurnPt.turnLineIndex)
                    {
                        for (int a = 0; a < cnt2; cnt2--)
                        {
                            ytList.Add(new vec3(ytList2[cnt2 - 1]));
                        }

                    }
                    else
                    {
                        //mulitple segments
                        vec3 tPoint = new vec3();
                        int turnCount = mf.bnd.bndList[startClosestTurnPt.turnLineNum].turnLine.Count;

                        //how many points from turnline do we add
                        int loops = Math.Abs(startClosestTurnPt.turnLineIndex - goalClosestTurnPt.turnLineIndex);

                        //are we crossing a border?
                        if (loops > (mf.bnd.bndList[startClosestTurnPt.turnLineNum].turnLine.Count / 2))
                        {
                            if (startClosestTurnPt.turnLineIndex < goalClosestTurnPt.turnLineIndex)
                            {
                                loops = (turnCount - goalClosestTurnPt.turnLineIndex) + startClosestTurnPt.turnLineIndex;
                            }
                            else
                            {
                                loops = (turnCount - startClosestTurnPt.turnLineIndex) + goalClosestTurnPt.turnLineIndex;
                            }
                        }

                        //count up - start with B which is next A
                        if (isFirstTurnLineSameWay)
                        {
                            for (int i = 0; i < loops; i++)
                            {
                                if ((startClosestTurnPt.turnLineIndex + 1) >= turnCount) startClosestTurnPt.turnLineIndex = -1;

                                tPoint = mf.bnd.bndList[startClosestTurnPt.turnLineNum].turnLine[startClosestTurnPt.turnLineIndex + 1];
                                startClosestTurnPt.turnLineIndex++;
                                if (startClosestTurnPt.turnLineIndex >= turnCount)
                                    startClosestTurnPt.turnLineIndex = 0;
                                ytList.Add(tPoint);
                            }
                        }
                        else //count down = start with A
                        {
                            for (int i = 0; i < loops; i++)
                            {
                                tPoint = mf.bnd.bndList[startClosestTurnPt.turnLineNum].turnLine[startClosestTurnPt.turnLineIndex];
                                startClosestTurnPt.turnLineIndex--;
                                if (startClosestTurnPt.turnLineIndex == -1)
                                    startClosestTurnPt.turnLineIndex = turnCount - 1;
                                ytList.Add(tPoint);
                            }
                        }

                        //add the out from ytList2
                        for (int a = 0; a < cnt2; cnt2--)
                        {
                            ytList.Add(new vec3(ytList2[cnt2 - 1]));
                        }
                    }

                    //AddCurveSequenceLines(mf.curve, nextCurve);

                    //fill in the gaps
                    double distance;

                    int cnt = ytList.Count;
                    for (int i = 1; i < cnt - 2; i++)
                    {
                        int j = i + 1;
                        if (j == cnt - 1) continue;
                        distance = glm.DistanceSquared(ytList[i], ytList[j]);
                        if (distance > 1)
                        {
                            vec3 pointB = new vec3((ytList[i].easting + ytList[j].easting) / 2.0,
                                (ytList[i].northing + ytList[j].northing) / 2.0, ytList[i].heading);

                            ytList.Insert(j, pointB);
                            cnt = ytList.Count;
                            i--;
                        }
                    }

                    //calculate the new points headings based on fore and aft of point - smoother turns
                    cnt = ytList.Count;
                    vec3[] arr = new vec3[cnt];
                    cnt -= 2;
                    ytList.CopyTo(arr);
                    ytList.Clear();

                    for (int i = 2; i < cnt; i++)
                    {
                        vec3 pt3 = new vec3(arr[i]);
                        pt3.heading = Math.Atan2(arr[i + 1].easting - arr[i - 1].easting,
                            arr[i + 1].northing - arr[i - 1].northing);
                        if (pt3.heading < 0) pt3.heading += glm.twoPI;
                        ytList.Add(pt3);
                    }

                    //check to close
                    if (glm.Distance(ytList[0], mf.pivotAxlePos) < 3)
                    {
                        FailCreate();
                        return false;
                    }

                    youTurnPhase = 10;
                    return true;
            }

            // just in case
            return false;
        }

        public bool FindCurveOutTurnPoint(CABCurve thisCurve, ref CABCurve nextCurve, CClose inPt, bool isTurnLineSameWay)
        {
            int a = isTurnLineSameWay ? 1 : -1;

            int turnLineIndex = inPt.turnLineIndex;
            int turnLineNum = inPt.turnLineNum;
            int stopTurnLineIndex = inPt.turnLineIndex - a;
            if (stopTurnLineIndex < 0) stopTurnLineIndex = mf.bnd.bndList[turnLineNum].turnLine.Count - 3;
            if (stopTurnLineIndex > mf.bnd.bndList[turnLineNum].turnLine.Count - 1) turnLineIndex = 3;


            for (; turnLineIndex != stopTurnLineIndex; turnLineIndex += a)
            {
                if (turnLineIndex < 0) turnLineIndex = mf.bnd.bndList[turnLineNum].turnLine.Count - 2; //AAA could be less than 0???
                if (turnLineIndex > mf.bnd.bndList[turnLineNum].turnLine.Count - 2) turnLineIndex = 0;

                for (int i = 0; i < nextCurve.curList.Count - 2; i++)
                {
                    int res = GetLineIntersection(
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].easting,
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].northing,
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex + 1].easting,
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex + 1].northing,

                                    nextCurve.curList[i].easting,
                                    nextCurve.curList[i].northing,
                                    nextCurve.curList[i + 1].easting,
                                    nextCurve.curList[i + 1].northing,
                                     ref iE, ref iN);
                    if (res == 1)
                    {
                        closestTurnPt = new CClose();
                        closestTurnPt.closePt.easting = iE;
                        closestTurnPt.closePt.northing = iN;
                        closestTurnPt.closePt.heading = nextCurve.curList[i].heading;
                        closestTurnPt.turnLineIndex = turnLineIndex;
                        closestTurnPt.curveIndex = i;
                        closestTurnPt.turnLineHeading = mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].heading;
                        closestTurnPt.turnLineNum = turnLineNum;
                        return true;
                    }
                }

                for (int i = 0; i < thisCurve.curList.Count - 2; i++)
                {
                    int res = GetLineIntersection(
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].easting,
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].northing,
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex + 1].easting,
                                    mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex + 1].northing,

                                    thisCurve.curList[i].easting,
                                    thisCurve.curList[i].northing,
                                    thisCurve.curList[i + 1].easting,
                                    thisCurve.curList[i + 1].northing,

                                     ref iE, ref iN);
                    if (res == 1)
                    {
                        if ((i < inPt.curveIndex && thisCurve.isHeadingSameWay) || (i > inPt.curveIndex && !thisCurve.isHeadingSameWay))
                        {
                            return false; //hitting the curve behind us
                        }
                        else if (i == inPt.curveIndex)
                        {
                            //do nothing hitting the curve at the same place as in
                        }
                        else
                        {
                            closestTurnPt = new CClose();
                            closestTurnPt.closePt.easting = iE;
                            closestTurnPt.closePt.northing = iN;
                            closestTurnPt.closePt.heading = thisCurve.curList[i].heading; //ändrad nyss till this curve
                            closestTurnPt.turnLineIndex = turnLineIndex;
                            closestTurnPt.curveIndex = i;
                            closestTurnPt.turnLineHeading = mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].heading;
                            closestTurnPt.turnLineNum = turnLineNum;
                            isOutSameCurve = true;
                            nextCurve = thisCurve;
                            return true;
                        }
                    }
                }
            }
            return false;
        }
        private bool FindInnerTurnPoints(vec3 fromPt, double inDirection, CClose refClosePt, bool isTurnLineSameWay)
        {
            double eP, nP;

            eP = fromPt.easting + Math.Sin(inDirection);
            nP = fromPt.northing + Math.Cos(inDirection);

            int a = isTurnLineSameWay ? 1 : -1;

            int turnLineIndex = refClosePt.turnLineIndex;
            int turnLineNum = refClosePt.turnLineNum;
            int stopTurnLineIndex = refClosePt.turnLineIndex - a;
            if (stopTurnLineIndex < 0) stopTurnLineIndex = mf.bnd.bndList[turnLineNum].turnLine.Count - 3;
            if (stopTurnLineIndex > mf.bnd.bndList[turnLineNum].turnLine.Count - 1) turnLineIndex = 3;


            for (; turnLineIndex != stopTurnLineIndex; turnLineIndex += a)
            {
                if (turnLineIndex < 0) turnLineIndex = mf.bnd.bndList[turnLineNum].turnLine.Count - 2;
                if (turnLineIndex > mf.bnd.bndList[turnLineNum].turnLine.Count - 2) turnLineIndex = 0;


                int res = GetLineIntersection(
                                mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].easting,
                                mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].northing,
                                mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex + 1].easting,
                                mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex + 1].northing,

                                fromPt.easting,
                                fromPt.northing, eP, nP, ref iE, ref iN);
                if (res == 1)
                {
                    closestTurnPt = new CClose();
                    closestTurnPt.closePt.easting = iE;
                    closestTurnPt.closePt.northing = iN;
                    closestTurnPt.closePt.heading = -1; //isnt needed but could be calculated
                    closestTurnPt.turnLineIndex = turnLineIndex;
                    closestTurnPt.curveIndex = -1;
                    closestTurnPt.turnLineHeading = mf.bnd.bndList[turnLineNum].turnLine[turnLineIndex].heading;
                    closestTurnPt.turnLineNum = turnLineNum;
                    return true;
                }

            }
            return false;
        }

        private bool FindCurveTurnPoint(CABCurve thisCurve)
        {
            //AAA Is updated but not tested....
            //find closet AB Curve point that will cross and go out of bounds
            int Count = mf.curve.isHeadingSameWay ? 1 : -1;
            int turnNum = 99;
            int j;

            closestTurnPt = new CClose();

            for (j = thisCurve.currentLocationIndex; j > 0 && j < thisCurve.curList.Count; j += Count)
            {
                int turnIndex = mf.bnd.IsPointInsideTurnArea(thisCurve.curList[j]);
                if (turnIndex != 0)
                {
                    closestTurnPt.curveIndex = j - Count;
                    closestTurnPt.turnLineNum = turnIndex;
                    turnNum = turnIndex;
                    break;
                }
            }

            if (turnNum < 0) //uturn will be on outer boundary turn
            {
                closestTurnPt.turnLineNum = 0;
                turnNum = 0;
            }
            else if (turnNum == 99)
            {
                //curve does not cross a boundary - oops
                isTurnCreationNotCrossingError = true;
                return false;
            }

            if (closestTurnPt.curveIndex == -1)
            {
                isTurnCreationNotCrossingError = true;
                return false;
            }


            for (int i = 0; i < mf.bnd.bndList[turnNum].turnLine.Count - 1; i++)
            {
                int res = GetLineIntersection(
                        mf.bnd.bndList[turnNum].turnLine[i].easting,
                        mf.bnd.bndList[turnNum].turnLine[i].northing,
                        mf.bnd.bndList[turnNum].turnLine[i + 1].easting,
                        mf.bnd.bndList[turnNum].turnLine[i + 1].northing,

                        thisCurve.curList[closestTurnPt.curveIndex].easting,
                        thisCurve.curList[closestTurnPt.curveIndex].northing,
                        thisCurve.curList[closestTurnPt.curveIndex + Count].easting,
                        thisCurve.curList[closestTurnPt.curveIndex + Count].northing,

                         ref iE, ref iN);

                if (res == 1)
                {
                    closestTurnPt.closePt.easting = iE;
                    closestTurnPt.closePt.northing = iN;

                    /*double hed = Math.Atan2(mf.bnd.bndList[turnNum].turnLine[i + 1].easting - mf.bnd.bndList[turnNum].turnLine[i].easting,
                        mf.bnd.bndList[turnNum].turnLine[i + 1].northing - mf.bnd.bndList[turnNum].turnLine[i].northing);
                    if (hed < 0) hed += glm.twoPI;
                    crossingheading = hed;*/
                    closestTurnPt.closePt.heading = thisCurve.curList[closestTurnPt.curveIndex].heading;
                    closestTurnPt.turnLineIndex = i;
                    closestTurnPt.turnLineNum = turnNum;
                    closestTurnPt.turnLineHeading = mf.bnd.bndList[turnNum].turnLine[i].heading;
                    if(!thisCurve.isHeadingSameWay && closestTurnPt.curveIndex > 0) closestTurnPt.curveIndex--;

                    break;
                }
            }

            return closestTurnPt.turnLineIndex != -1 && closestTurnPt.curveIndex != -1;
        }

        public int GetLineIntersection(double p0x, double p0y, double p1x, double p1y,
                double p2x, double p2y, double p3x, double p3y, ref double iEast, ref double iNorth)
        {
            double s1x, s1y, s2x, s2y;
            s1x = p1x - p0x;
            s1y = p1y - p0y;

            s2x = p3x - p2x;
            s2y = p3y - p2y;

            double s, t;
            s = (-s1y * (p0x - p2x) + s1x * (p0y - p2y)) / (-s2x * s1y + s1x * s2y);

            if (s >= 0 && s <= 1)
            {
                //check oher side
                t = (s2x * (p0y - p2y) - s2y * (p0x - p2x)) / (-s2x * s1y + s1x * s2y);
                if (t >= 0 && t <= 1)
                {
                    // Collision detected
                    iEast = p0x + (t * s1x);
                    iNorth = p0y + (t * s1y);
                    return 1;
                }
            }

            return 0; // No collision
        }

        public void AddSequenceLines(double head)
        {
            //how many points striaght out
            double lenny = 8;

            vec3 pt;
            for (int a = 0; a < lenny; a++)
            {
                pt.easting = ytList[0].easting + (Math.Sin(head) * 0.511);
                pt.northing = ytList[0].northing + (Math.Cos(head) * 0.511);
                pt.heading = ytList[0].heading;
                ytList.Insert(0, pt);
            }

            int count = ytList.Count;

            for (int i = 1; i <= lenny; i++)
            {
                pt.easting = ytList[count - 1].easting + (Math.Sin(head) * i * 0.511);
                pt.northing = ytList[count - 1].northing + (Math.Cos(head) * i * 0.511);
                pt.heading = head;
                ytList.Add(pt);
            }

            double distancePivotToTurnLine;
            count = ytList.Count;
            for (int i = 0; i < count; i += 2)
            {
                distancePivotToTurnLine = glm.DistanceSquared(ytList[i], mf.pivotAxlePos);
                if (distancePivotToTurnLine > 6)
                {
                    isTurnCreationTooClose = false;
                }
                else
                {
                    //set the flag to Critical stop machine
                    FailCreate();
                    break;
                }
            }
        }

        public void FailCreate()
        {
            //fail
            isOutOfBounds = true;
            isTurnCreationTooClose = true;
            mf.mc.isOutOfBounds = true;
            youTurnPhase = 11;
        }

        public bool KStyleTurn(bool isTurnLeft, double headAB)
        {
            //grab the pure pursuit point right on ABLine
            vec3 onPurePoint = new vec3(mf.ABLine.rEastAB, mf.ABLine.rNorthAB, 0);

            //how far are we from any turn boundary
            FindABTurnPoint(onPurePoint);

            //or did we lose the turnLine - we are on the highway cuz we left the outer/inner turn boundary
            if (closestTurnPt.turnLineIndex != -1)
            {
                mf.distancePivotToTurnLine = glm.Distance(mf.pivotAxlePos, closestTurnPt.closePt);
            }
            else
            {
                //Full emergency stop code goes here, it thinks its auto turn, but its not!
                return false;
            }

            //delta between AB heading and boundary closest point heading
            boundaryAngleOffPerpendicular = Math.PI - Math.Abs(Math.Abs(closestTurnPt.closePt.heading - headAB) - Math.PI);
            boundaryAngleOffPerpendicular -= glm.PIBy2;
            boundaryAngleOffPerpendicular *= -1;
            if (boundaryAngleOffPerpendicular > 1.25) boundaryAngleOffPerpendicular = 1.25;
            if (boundaryAngleOffPerpendicular < -1.25) boundaryAngleOffPerpendicular = -1.25;

            //for calculating innner circles of turn
            double tangencyAngle = (glm.PIBy2 - Math.Abs(boundaryAngleOffPerpendicular)) * 0.5;
            double tangencyFactor = Math.Tan(tangencyAngle);

            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.width - mf.tool.overlap) * rowSkipsWidth + (isYouTurnRight ? -mf.tool.offset * 2.0 : mf.tool.offset * 2.0);

            //double turnRadius = turnOffset / Math.Cos(boundaryAngleOffPerpendicular);
            double turnRadius = youTurnRadius;

            isHeadingSameWay = mf.ABLine.isHeadingSameWay;
            double head = mf.ABLine.abHeading;
            if (!isHeadingSameWay) head += Math.PI;

            double abLineHeading = head;

            //intersection of ABline and boundary
            rEastYT = closestTurnPt.closePt.easting;
            rNorthYT = closestTurnPt.closePt.northing;

            vec3 oneStart = new vec3
            {
                easting = rEastYT,
                northing = rNorthYT,
                heading = 0
            };

            vec3 oneEnd = new vec3
            {
                easting = rEastYT,
                northing = rNorthYT,
                heading = 0
            };

            vec3 twoStart = new vec3
            {
                easting = rEastYT,
                northing = rNorthYT,
                heading = 0
            };

            vec3 twoEnd = new vec3
            {
                easting = rEastYT,
                northing = rNorthYT,
                heading = 0
            };

            if (boundaryAngleOffPerpendicular < 0)
            {
                //which is actually left
                if (isYouTurnRight)
                {
                    turnRadius = (youTurnRadius * tangencyFactor);//short
                }
                else
                {
                    turnRadius = (youTurnRadius / tangencyFactor); //long
                }
            }
            else
            {
                //which is actually left
                if (isYouTurnRight)
                {
                    turnRadius = (youTurnRadius / tangencyFactor); //long
                }
                else
                {
                    turnRadius = (youTurnRadius * tangencyFactor); //short
                }
            }

            //move the start back away from turn line youTurnRadius distance based on tangency
            oneStart.easting -= (Math.Sin(head) * turnRadius);
            oneStart.northing -= (Math.Cos(head) * turnRadius);
            oneStart.heading = head;

            double arcAngle = glm.PIBy2;
            // move the goal left or right at 90 degrees
            if (!isTurnLeft) //means going right
            {
                head += glm.PIBy2;
                arcAngle -= boundaryAngleOffPerpendicular;
                //head -= angle;
            }
            else
            {
                head -= glm.PIBy2;
                arcAngle += boundaryAngleOffPerpendicular;
                //head += angle;
            }

            if (head < -Math.PI) head += glm.twoPI;
            if (head > Math.PI) head -= glm.twoPI;

            //point to set next AB Line via lateral
            pt3TurnNewAB.easting = oneEnd.easting + (Math.Sin(head) * mf.tool.width);
            pt3TurnNewAB.northing = oneEnd.northing + (Math.Cos(head) * mf.tool.width);

            oneEnd.easting = oneStart.easting + Math.Sin(head) * youTurnRadius;
            oneEnd.northing = oneStart.northing + Math.Cos(head) * youTurnRadius;

            twoEnd.heading = 0; // - angle;
            oneEnd.heading = 0;

            //two.easting = pt3TurnNewAB.easting - (Math.Sin(head) * mf.vehicle.minTurningRadius);
            //two.northing = pt3TurnNewAB.northing - (Math.Cos(head) * mf.vehicle.minTurningRadius);

            double r = youTurnRadius;
            int numSegments = (int)(arcAngle * 16);

            double theta = arcAngle / (double)(numSegments - 1);

            double tanFactor = Math.Tan(theta);
            double radialFactor = Math.Cos(theta);

            double startAngle = abLineHeading;
            if (!isYouTurnRight)
            {
                startAngle -= 1.57;
            }
            else
            {
                startAngle += 1.57;
            }

            if (startAngle < -Math.PI) startAngle += glm.twoPI;
            if (startAngle > Math.PI) startAngle -= glm.twoPI;

            double x = r * Math.Sin(startAngle);
            double y = r * Math.Cos(startAngle);

            vec3 pt;
            for (int ii = 0; ii < numSegments; ii++)
            {
                //glVertex2f(x + cx, y + cy);
                pt.easting = x + oneEnd.easting;
                pt.northing = y + oneEnd.northing;
                pt.heading = 0;

                ytList.Add(pt);

                double tx;
                double ty;
                if (!isYouTurnRight)
                {
                    tx = y;
                    ty = -x;
                }
                else
                {
                    tx = -y;
                    ty = x;
                }

                x += tx * tanFactor;
                y += ty * tanFactor;

                x *= radialFactor;
                y *= radialFactor;
            }

            for (int a = 0; a < 8; a++)
            {
                pt.easting = ytList[0].easting - (Math.Sin(abLineHeading) * 0.5);
                pt.northing = ytList[0].northing - (Math.Cos(abLineHeading) * 0.5);
                pt.heading = ytList[0].heading;
                ytList.Insert(0, pt);
            }

            //we are following the turn line now.
            head -= boundaryAngleOffPerpendicular;

            //from end of turn to over new AB a bit
            //double twoEndExtension = mf.tool.width + mf.vehicle.wheelbase - youTurnRadius;
            //if (mf.tool.width < turnRadius) twoEndExtension = mf.vehicle.wheelbase;
            int twoEndExtension = (int)(mf.tool.width * 5);

            //add the tail to first turn
            int count = ytList.Count;
            for (int i = 1; i <= twoEndExtension; i++)
            {
                pt.easting = ytList[count - 1].easting + (Math.Sin(head) * i * 0.5);
                pt.northing = ytList[count - 1].northing + (Math.Cos(head) * i * 0.5);
                pt.heading = 0;
                ytList.Add(pt);
            }

            //calculate line headings
            vec3[] arr = new vec3[ytList.Count];
            ytList.CopyTo(arr);
            ytList.Clear();

            //headings of line one
            for (int i = 0; i < arr.Length - 1; i++)
            {
                arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                ytList.Add(arr[i]);
            }

            //LINE TWO - use end of line one for end of line two, both same direction bit longer
            twoEnd.easting = ytList[ytList.Count - 1].easting;
            twoEnd.northing = ytList[ytList.Count - 1].northing;
            twoEnd.heading = ytList[ytList.Count - 1].heading;

            if (twoEnd.heading < -Math.PI) twoEnd.heading += glm.twoPI;
            if (twoEnd.heading > Math.PI) twoEnd.heading -= glm.twoPI;

            //straight line
            twoStart = twoEnd;

            ////backing up to this point
            twoStart.easting -= (Math.Sin(head) * 40);
            twoStart.northing -= (Math.Cos(head) * 40);

            pt3ListSecondLine?.Clear();
            pt = twoStart;
            pt3ListSecondLine.Add(pt);

            //from start to end
            count = pt3ListSecondLine.Count;
            for (int i = 1; i <= 80; i++)
            {
                pt.easting = pt3ListSecondLine[count - 1].easting + (Math.Sin(head) * i * 0.5);
                pt.northing = pt3ListSecondLine[count - 1].northing + (Math.Cos(head) * i * 0.5);
                pt.heading = 0;
                pt3ListSecondLine.Add(pt);
            }

            if (pt3ListSecondLine.Count != 0)
            {
                youTurnPhase = 10;
                return true;
            }
            else
            {
                return false;
            }

        }

        public void FindABTurnPoint(vec3 fromPt)
        {
            double eP = fromPt.easting;
            double nP = fromPt.northing;
            double eAB, nAB;
            turnClosestList?.Clear();

            CClose cClose;

            if (mf.ABLine.isHeadingSameWay)
            {
                eAB = mf.ABLine.currentLinePtB.easting;
                nAB = mf.ABLine.currentLinePtB.northing;
            }
            else
            {
                eAB = mf.ABLine.currentLinePtA.easting;
                nAB = mf.ABLine.currentLinePtA.northing;
            }

            turnClosestList.Clear();

            for (int j = 0; j < mf.bnd.bndList.Count; j++)
            {
                for (int i = 0; i < mf.bnd.bndList[j].turnLine.Count - 1; i++)
                {
                    int res = GetLineIntersection(
                        mf.bnd.bndList[j].turnLine[i].easting,
                        mf.bnd.bndList[j].turnLine[i].northing,
                        mf.bnd.bndList[j].turnLine[i + 1].easting,
                        mf.bnd.bndList[j].turnLine[i + 1].northing,
                        eP, nP, eAB, nAB, ref iE, ref iN
                    );

                    if (res == 1)
                    {
                        cClose = new CClose();
                        cClose.closePt.easting = iE;
                        cClose.closePt.northing = iN;

                        double hed = Math.Atan2(mf.bnd.bndList[j].turnLine[i + 1].easting - mf.bnd.bndList[j].turnLine[i].easting,
                            mf.bnd.bndList[j].turnLine[i + 1].northing - mf.bnd.bndList[j].turnLine[i].northing);
                        if (hed < 0) hed += glm.twoPI;
                        cClose.closePt.heading = hed;
                        cClose.turnLineNum = j;
                        cClose.turnLineIndex = i;

                        turnClosestList.Add(new CClose(cClose));
                    }
                }
            }

            //determine closest point
            double minDistance = double.MaxValue;

            if (turnClosestList.Count > 0)
            {
                for (int i = 0; i < turnClosestList.Count; i++)
                {
                    double dist = (((fromPt.easting - turnClosestList[i].closePt.easting) * (fromPt.easting - turnClosestList[i].closePt.easting))
                                    + ((fromPt.northing - turnClosestList[i].closePt.northing) * (fromPt.northing - turnClosestList[i].closePt.northing)));

                    if (minDistance >= dist)
                    {

                        minDistance = dist;
                        closestTurnPt = new CClose(turnClosestList[i]);
                    }
                }
            }
        }

        private List<vec3> MoveTurnInsideTurnLine(List<vec3> uTurnList, double head, bool deleteSecondHalf, bool invertHeading)
        {
            //step 1 make array out of the list so that we can modify the position
            double cosHead = Math.Cos(head);
            double sinHead = Math.Sin(head);
            int cnt = uTurnList.Count;
            vec3[] arr2 = new vec3[cnt];
            uTurnList.CopyTo(arr2);
            uTurnList.Clear();

            semiCircleIndex = -1;
            //step 2 move the turn inside with steps of 1 meter
            bool pointOutOfBnd = isOutOfBounds;
            int j = 0;
            int stopIfWayOut = 0;
            while (pointOutOfBnd)
            {
                stopIfWayOut++;
                pointOutOfBnd = false;
                mf.distancePivotToTurnLine = glm.Distance(arr2[0], mf.pivotAxlePos);

                for (int i = 0; i < cnt; i++)
                {
                    arr2[i].easting -= (sinHead);
                    arr2[i].northing -= (cosHead);
                }

                for (; j < cnt; j += 1)
                {
                    if (mf.bnd.IsPointInsideTurnArea(arr2[j]) != 0)
                    {
                        pointOutOfBnd = true;
                        if (j > 0) j--;
                        break;
                    }
                }

                if (stopIfWayOut == 1000 || (mf.distancePivotToTurnLine < 3))
                {
                    //for some reason it doesn't go inside boundary, return empty list
                    return uTurnList;
                }
            }

            //step 3, we ar now inside turnline, move the turn forward until it hits the turnfence in steps of 0.1 meters
            while (!pointOutOfBnd)
            {
                for (int i = 0; i < cnt; i++)
                {
                    arr2[i].easting += (sinHead * 0.1);
                    arr2[i].northing += (cosHead * 0.1);
                }

                for (int a = 0; a < cnt; a++)
                {
                    if (mf.bnd.IsPointInsideTurnArea(arr2[a]) != 0)
                    {
                        semiCircleIndex = a;
                        pointOutOfBnd = true;
                        break;

                    }
                }
            }

            //step 4, Should we delete the points after the one that is outside? and where the points made in the wrong direction?
            for (int i = 0; i < cnt; i++)
            {
                if (i == semiCircleIndex && deleteSecondHalf)
                    break;
                if (invertHeading) arr2[i].heading += Math.PI;
                if (arr2[i].heading >= glm.twoPI) arr2[i].heading -= glm.twoPI;
                else if (arr2[i].heading < 0) arr2[i].heading += glm.twoPI;
                uTurnList.Add(arr2[i]);
            }

            //we have succesfully moved the turn inside
            isOutOfBounds = false;

            //if empty - no creation.
            return uTurnList;

        }

        public void SmoothYouTurn(int smPts)
        {
            //count the reference list of original curve
            int cnt = ytList.Count;

            //the temp array
            vec3[] arr = new vec3[cnt];

            //read the points before and after the setpoint
            for (int s = 0; s < smPts / 2; s++)
            {
                arr[s].easting = ytList[s].easting;
                arr[s].northing = ytList[s].northing;
                arr[s].heading = ytList[s].heading;
            }

            for (int s = cnt - (smPts / 2); s < cnt; s++)
            {
                arr[s].easting = ytList[s].easting;
                arr[s].northing = ytList[s].northing;
                arr[s].heading = ytList[s].heading;
            }

            //average them - center weighted average
            for (int i = smPts / 2; i < cnt - (smPts / 2); i++)
            {
                for (int j = -smPts / 2; j < smPts / 2; j++)
                {
                    arr[i].easting += ytList[j + i].easting;
                    arr[i].northing += ytList[j + i].northing;
                }
                arr[i].easting /= smPts;
                arr[i].northing /= smPts;
                arr[i].heading = ytList[i].heading;
            }

            ytList?.Clear();

            //calculate new headings on smoothed line
            for (int i = 1; i < cnt - 1; i++)
            {
                arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                ytList.Add(arr[i]);
            }
        }

        //called to initiate turn
        public void YouTurnTrigger()
        {
            //trigger pulled
            isYouTurnTriggered = true;

            if (alternateSkips && rowSkipsWidth2 > 1)
            {
                if (--turnSkips == 0)
                {
                    isYouTurnRight = !isYouTurnRight;
                    turnSkips = rowSkipsWidth2 * 2 - 1;
                }
                else if (previousBigSkip = !previousBigSkip)
                    rowSkipsWidth = rowSkipsWidth2 - 1;
                else
                    rowSkipsWidth = rowSkipsWidth2;
            }
            else isYouTurnRight = !isYouTurnRight;

            if (uTurnStyle == 0)
            {
                mf.guidanceLookPos.easting = ytList[ytList.Count - 1].easting;
                mf.guidanceLookPos.northing = ytList[ytList.Count - 1].northing;
            }
            else if (uTurnStyle == 1)
            {
                mf.guidanceLookPos.easting = pt3TurnNewAB.easting;
                mf.guidanceLookPos.northing = pt3TurnNewAB.northing;

                pt3Phase = 0;
            }

            if (mf.trk.idx > -1 && mf.trk.gArr.Count > 0)
            {
                if (mf.trk.gArr[mf.trk.idx].mode == (int)TrackMode.AB)
                {
                    mf.ABLine.isLateralTriggered = true;
                    mf.ABLine.isABValid = false;
                }
                else
                {
                    mf.curve.isLateralTriggered = true;
                    mf.curve.isCurveValid = false;
                }
            }
        }

        //Normal copmpletion of youturn
        public void CompleteYouTurn()
        {
            isYouTurnTriggered = false;
            ResetCreatedYouTurn();
            mf.sounds.isBoundAlarming = false;
        }

        public void Set_Alternate_skips()
        {
            rowSkipsWidth2 = rowSkipsWidth;
            turnSkips = rowSkipsWidth2 * 2 - 1;
            previousBigSkip = false;
        }

        //something went seriously wrong so reset everything
        public void ResetYouTurn()
        {
            //fix you turn
            isYouTurnTriggered = false;
            ytList?.Clear();
            ResetCreatedYouTurn();
            mf.sounds.isBoundAlarming = false;
            isTurnCreationTooClose = false;
            isTurnCreationNotCrossingError = false;
            mf.p_239.pgn[mf.p_239.uturn] = 0;
        }

        public void ResetCreatedYouTurn()
        {
            youTurnPhase = 0;
            ytList?.Clear();
            pt3ListSecondLine?.Clear();
            pt3Phase = 0;
            mf.p_239.pgn[mf.p_239.uturn] = 0;
        }

        public void BuildManualYouLateral(bool isTurnLeft)
        {
            double head;
            //point on AB line closest to pivot axle point from ABLine PurePursuit
            if (mf.trk.idx > -1 && mf.trk.gArr.Count > 0)
            {
                if (mf.trk.gArr[mf.trk.idx].mode == (int)TrackMode.AB)
                {
                    rEastYT = mf.ABLine.rEastAB;
                    rNorthYT = mf.ABLine.rNorthAB;
                    isHeadingSameWay = mf.ABLine.isHeadingSameWay;
                    head = mf.ABLine.abHeading;
                    mf.ABLine.isLateralTriggered = true;
                }
                else
                {
                    rEastYT = mf.curve.rEastCu;
                    rNorthYT = mf.curve.rNorthCu;
                    isHeadingSameWay = mf.curve.isHeadingSameWay;
                    head = mf.curve.manualUturnHeading;
                    mf.curve.isLateralTriggered = true;
                }
            }
            else return;

            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.width - mf.tool.overlap); //remove rowSkips

            //if its straight across it makes 2 loops instead so goal is a little lower then start
            if (!isHeadingSameWay) head += Math.PI;

            //move the start forward 2 meters, this point is critical to formation of uturn
            rEastYT += (Math.Sin(head) * 2);
            rNorthYT += (Math.Cos(head) * 2);

            if (isTurnLeft)
            {
                mf.guidanceLookPos.easting = rEastYT + (Math.Cos(-head) * turnOffset);
                mf.guidanceLookPos.northing = rNorthYT + (Math.Sin(-head) * turnOffset);
            }
            else
            {
                mf.guidanceLookPos.easting = rEastYT - (Math.Cos(-head) * turnOffset);
                mf.guidanceLookPos.northing = rNorthYT - (Math.Sin(-head) * turnOffset);
            }

            mf.ABLine.isABValid = false;
            mf.curve.isCurveValid = false;
        }

        //build the points and path of youturn to be scaled and transformed
        public void BuildManualYouTurn(bool isTurnLeft, bool isTurnButtonTriggered)
        {
            isYouTurnTriggered = true;

            double head;
            //point on AB line closest to pivot axle point from ABLine PurePursuit
            if (mf.trk.idx > -1 && mf.trk.gArr.Count > 0)
            {
                if (mf.trk.gArr[mf.trk.idx].mode == (int)TrackMode.AB)
                {
                    rEastYT = mf.ABLine.rEastAB;
                    rNorthYT = mf.ABLine.rNorthAB;
                    isHeadingSameWay = mf.ABLine.isHeadingSameWay;
                    head = mf.ABLine.abHeading;
                    mf.ABLine.isLateralTriggered = true;
                }

                else
                {
                    rEastYT = mf.curve.rEastCu;
                    rNorthYT = mf.curve.rNorthCu;
                    isHeadingSameWay = mf.curve.isHeadingSameWay;
                    head = mf.curve.manualUturnHeading;
                    mf.curve.isLateralTriggered = true;
                }
            }
            else return;

            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.width - mf.tool.overlap) * rowSkipsWidth + (isTurnLeft ? mf.tool.offset * 2.0 : -mf.tool.offset * 2.0);

            CDubins dubYouTurnPath = new CDubins();
            CDubins.turningRadius = youTurnRadius;

            //if its straight across it makes 2 loops instead so goal is a little lower then start
            if (!isHeadingSameWay) head += 3.14;
            else head -= 0.01;

            //move the start forward 2 meters, this point is critical to formation of uturn
            rEastYT += (Math.Sin(head) * 4);
            rNorthYT += (Math.Cos(head) * 4);

            //now we have our start point
            vec3 start = new vec3(rEastYT, rNorthYT, head);
            vec3 goal = new vec3();

            //now we go the other way to turn round
            head -= Math.PI;
            if (head < 0) head += glm.twoPI;

            //set up the goal point for Dubins
            goal.heading = head;
            if (isTurnButtonTriggered)
            {
                if (isTurnLeft)
                {
                    goal.easting = rEastYT - (Math.Cos(-head) * turnOffset);
                    goal.northing = rNorthYT - (Math.Sin(-head) * turnOffset);
                }
                else
                {
                    goal.easting = rEastYT + (Math.Cos(-head) * turnOffset);
                    goal.northing = rNorthYT + (Math.Sin(-head) * turnOffset);
                }
            }

            //generate the turn points
            ytList = dubYouTurnPath.GenerateDubins(start, goal);

            mf.guidanceLookPos.easting = ytList[ytList.Count - 1].easting;
            mf.guidanceLookPos.northing = ytList[ytList.Count - 1].northing;

            //vec3 pt;
            //for (double a = 0; a < 2; a += 0.2)
            //{
            //    pt.easting = ytList[0].easting + (Math.Sin(head) * a);
            //    pt.northing = ytList[0].northing + (Math.Cos(head) * a);
            //    pt.heading = ytList[0].heading;
            //    ytList.Insert(0, pt);
            //}

            //int count = ytList.Count;

            //for (double i = 0.2; i <= 7; i += 0.2)
            //{
            //    pt.easting = ytList[count - 1].easting + (Math.Sin(head) * i);
            //    pt.northing = ytList[count - 1].northing + (Math.Cos(head) * i);
            //    pt.heading = head;
            //    ytList.Add(pt);
            //}

            mf.ABLine.isABValid = false;
            mf.curve.isCurveValid = false;
        }

        public int onA;

        //determine distance from youTurn guidance line
        public bool DistanceFromYouTurnLine()
        {
            //grab a copy from main - the steer position
            double minDistA = 1000000, minDistB = 1000000;
            int ptCount = ytList.Count;

            if (ptCount > 0)
            {
                if (mf.isStanleyUsed)
                {
                    pivot = mf.steerAxlePos;

                    //find the closest 2 points to current fix
                    for (int t = 0; t < ptCount; t++)
                    {
                        double dist = ((pivot.easting - ytList[t].easting) * (pivot.easting - ytList[t].easting))
                                        + ((pivot.northing - ytList[t].northing) * (pivot.northing - ytList[t].northing));
                        if (dist < minDistA)
                        {
                            minDistB = minDistA;
                            B = A;
                            minDistA = dist;
                            A = t;
                        }
                        else if (dist < minDistB)
                        {
                            minDistB = dist;
                            B = t;
                        }
                    }

                    if (minDistA > 16)
                    {
                        CompleteYouTurn();
                        return false;
                    }

                    //just need to make sure the points continue ascending or heading switches all over the place
                    if (A > B)
                    {
                        (B, A) = (A, B);
                    }

                    //minDistA = 100;
                    //int closestPt = 0;
                    //for (int i = 0; i < ptCount; i++)
                    //{
                    //    double distancePiv = glm.DistanceSquared(ytList[i], pivot);
                    //    if (distancePiv < minDistA)
                    //    {
                    //        minDistA = distancePiv;
                    //    }
                    //}

                    //feed backward to turn slower to keep pivot on
                    A -= 7;
                    if (A < 0)
                    {
                        A = 0;
                    }
                    B = A + 1;

                    //return and reset if too far away or end of the line
                    if (B >= ptCount - 8)
                    {
                        CompleteYouTurn();
                        return false;
                    }

                    //get the distance from currently active AB line, precalc the norm of line
                    double dx = ytList[B].easting - ytList[A].easting;
                    double dz = ytList[B].northing - ytList[A].northing;
                    if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dz) < Double.Epsilon) return false;

                    double abHeading = ytList[A].heading;

                    //how far from current AB Line is steer point 90 degrees from steer position
                    distanceFromCurrentLine = ((dz * pivot.easting) - (dx * pivot.northing) + (ytList[B].easting
                                * ytList[A].northing) - (ytList[B].northing * ytList[A].easting))
                                    / Math.Sqrt((dz * dz) + (dx * dx));

                    //Calc point on ABLine closest to current position and 90 degrees to segment heading
                    double U = (((pivot.easting - ytList[A].easting) * dx)
                                + ((pivot.northing - ytList[A].northing) * dz))
                                / ((dx * dx) + (dz * dz));

                    //critical point used as start for the uturn path - critical
                    rEastYT = ytList[A].easting + (U * dx);
                    rNorthYT = ytList[A].northing + (U * dz);

                    //the first part of stanley is to extract heading error
                    double abFixHeadingDelta = (pivot.heading - abHeading);

                    //Fix the circular error - get it from -Pi/2 to Pi/2
                    if (abFixHeadingDelta > Math.PI) abFixHeadingDelta -= Math.PI;
                    else if (abFixHeadingDelta < Math.PI) abFixHeadingDelta += Math.PI;
                    if (abFixHeadingDelta > glm.PIBy2) abFixHeadingDelta -= Math.PI;
                    else if (abFixHeadingDelta < -glm.PIBy2) abFixHeadingDelta += Math.PI;

                    if (mf.isReverse) abFixHeadingDelta *= -1;
                    //normally set to 1, less then unity gives less heading error.
                    abFixHeadingDelta *= mf.vehicle.stanleyHeadingErrorGain;
                    if (abFixHeadingDelta > 0.74) abFixHeadingDelta = 0.74;
                    if (abFixHeadingDelta < -0.74) abFixHeadingDelta = -0.74;

                    //the non linear distance error part of stanley
                    steerAngleYT = Math.Atan((distanceFromCurrentLine * mf.vehicle.stanleyDistanceErrorGain) / ((mf.avgSpeed * 0.277777) + 1));

                    //clamp it to max 42 degrees
                    if (steerAngleYT > 0.74) steerAngleYT = 0.74;
                    if (steerAngleYT < -0.74) steerAngleYT = -0.74;

                    //add them up and clamp to max in vehicle settings
                    steerAngleYT = glm.toDegrees((steerAngleYT + abFixHeadingDelta) * -1.0);
                    if (steerAngleYT < -mf.vehicle.maxSteerAngle) steerAngleYT = -mf.vehicle.maxSteerAngle;
                    if (steerAngleYT > mf.vehicle.maxSteerAngle) steerAngleYT = mf.vehicle.maxSteerAngle;
                }
                else
                {
                    pivot = mf.pivotAxlePos;

                    //find the closest 2 points to current fix
                    for (int t = 0; t < ptCount; t++)
                    {
                        double dist = ((pivot.easting - ytList[t].easting) * (pivot.easting - ytList[t].easting))
                                        + ((pivot.northing - ytList[t].northing) * (pivot.northing - ytList[t].northing));
                        if (dist < minDistA)
                        {
                            minDistB = minDistA;
                            B = A;
                            minDistA = dist;
                            A = t;
                        }
                        else if (dist < minDistB)
                        {
                            minDistB = dist;
                            B = t;
                        }
                    }

                    //just need to make sure the points continue ascending or heading switches all over the place
                    if (A > B)
                    {
                        (B, A) = (A, B);
                    }

                    onA = A;
                    double distancePiv = glm.Distance(ytList[A], pivot);

                    if (distancePiv > 2 || (B >= ptCount - 1))
                    {
                        CompleteYouTurn();
                        return false;
                    }

                    //get the distance from currently active AB line
                    double dx = ytList[B].easting - ytList[A].easting;
                    double dz = ytList[B].northing - ytList[A].northing;

                    if (Math.Abs(dx) < double.Epsilon && Math.Abs(dz) < double.Epsilon) return false;

                    //how far from current AB Line is fix
                    distanceFromCurrentLine = ((dz * pivot.easting) - (dx * pivot.northing) + (ytList[B].easting
                                * ytList[A].northing) - (ytList[B].northing * ytList[A].easting))
                                    / Math.Sqrt((dz * dz) + (dx * dx));

                    // ** Pure pursuit ** - calc point on ABLine closest to current position
                    double U = (((pivot.easting - ytList[A].easting) * dx)
                                + ((pivot.northing - ytList[A].northing) * dz))
                                / ((dx * dx) + (dz * dz));

                    rEastYT = ytList[A].easting + (U * dx);
                    rNorthYT = ytList[A].northing + (U * dz);

                    //sharp turns on you turn.
                    //update base on autosteer settings and distance from line
                    double goalPointDistance = 0.8 * mf.vehicle.UpdateGoalPointDistance();

                    isHeadingSameWay = true;
                    bool ReverseHeading = !mf.isReverse;

                    int count = ReverseHeading ? 1 : -1;
                    vec3 start = new vec3(rEastYT, rNorthYT, 0);
                    double distSoFar = 0;

                    for (int i = ReverseHeading ? B : A; i < ptCount && i >= 0; i += count)
                    {
                        // used for calculating the length squared of next segment.
                        double tempDist = glm.Distance(start, ytList[i]);

                        //will we go too far?
                        if ((tempDist + distSoFar) > goalPointDistance)
                        {
                            double j = (goalPointDistance - distSoFar) / tempDist; // the remainder to yet travel

                            goalPointYT.easting = (((1 - j) * start.easting) + (j * ytList[i].easting));
                            goalPointYT.northing = (((1 - j) * start.northing) + (j * ytList[i].northing));
                            break;
                        }
                        else distSoFar += tempDist;
                        start = ytList[i];
                        if (i == ptCount - 1)//goalPointDistance is longer than remaining u-turn
                        {
                            CompleteYouTurn();
                            return false;
                        }

                        if (pt3Phase == 1 && i < 2)
                        {
                            CompleteYouTurn();
                            return false;
                        }

                        if (uTurnStyle == 1 && pt3Phase == 0 && isLastFrameForward && mf.isReverse)
                        {
                            ytList.Clear();
                            ytList.AddRange(pt3ListSecondLine);
                            pt3Phase++;
                            return true;
                        }

                        if (uTurnStyle == 1 && pt3Phase == 1 && !isLastFrameForward && !mf.isReverse)
                        {
                            CompleteYouTurn();
                            return false;
                        }

                        isLastFrameForward = mf.isReverse;
                    }

                    //calc "D" the distance from pivot axle to lookahead point
                    double goalPointDistanceSquared = glm.DistanceSquared(goalPointYT.northing, goalPointYT.easting, pivot.northing, pivot.easting);

                    //calculate the the delta x in local coordinates and steering angle degrees based on wheelbase
                    double localHeading = glm.twoPI - mf.fixHeading;
                    ppRadiusYT = goalPointDistanceSquared / (2 * (((goalPointYT.easting - pivot.easting) * Math.Cos(localHeading)) + ((goalPointYT.northing - pivot.northing) * Math.Sin(localHeading))));

                    steerAngleYT = glm.toDegrees(Math.Atan(2 * (((goalPointYT.easting - pivot.easting) * Math.Cos(localHeading))
                        + ((goalPointYT.northing - pivot.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase / goalPointDistanceSquared));

                    if (steerAngleYT < -mf.vehicle.maxSteerAngle) steerAngleYT = -mf.vehicle.maxSteerAngle;
                    if (steerAngleYT > mf.vehicle.maxSteerAngle) steerAngleYT = mf.vehicle.maxSteerAngle;

                    if (ppRadiusYT < -500) ppRadiusYT = -500;
                    if (ppRadiusYT > 500) ppRadiusYT = 500;

                    radiusPointYT.easting = pivot.easting + (ppRadiusYT * Math.Cos(localHeading));
                    radiusPointYT.northing = pivot.northing + (ppRadiusYT * Math.Sin(localHeading));

                    //distance is negative if on left, positive if on right
                    if (!isHeadingSameWay)
                        distanceFromCurrentLine *= -1.0;
                }

                //used for smooth mode
                mf.vehicle.modeActualXTE = (distanceFromCurrentLine);

                //Convert to centimeters
                mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLine * 1000.0, MidpointRounding.AwayFromZero);
                mf.guidanceLineSteerAngle = (short)(steerAngleYT * 100);
                return true;
            }
            else
            {
                CompleteYouTurn();
                return false;
            }
        }

        //Duh.... What does this do....
        public void DrawYouTurn()
        {
            GL.PointSize(12.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.73f, 1.0f);
            GL.Vertex3(inClosestTurnPt.closePt.easting, inClosestTurnPt.closePt.northing, 0);
            GL.Color3(0.395f, 0.925f, 0.30f);
            GL.Vertex3(outClosestTurnPt.closePt.easting, outClosestTurnPt.closePt.northing, 0);
            GL.End();
            GL.PointSize(1.0f);

            int ptCount = ytList.Count;
            if (ptCount < 3) return;

            GL.PointSize(mf.ABLine.lineWidth + 2);

            if (isYouTurnTriggered)
                GL.Color3(0.95f, 0.5f, 0.95f);
            else if (isOutOfBounds)
                GL.Color3(0.9495f, 0.395f, 0.325f);
            else
                GL.Color3(0.395f, 0.925f, 0.30f);

            GL.Begin(PrimitiveType.Points);
            for (int i = 0; i < ptCount; i++)
            {
                GL.Vertex3(ytList[i].easting, ytList[i].northing, 0);
            }
            GL.End();

            if (nextCurve != null)
            {
                GL.Begin(PrimitiveType.Points);
                GL.Color3(0.95f, 0.41f, 0.980f);
               for (int i = 0; i < nextCurve.curList.Count; i++)
                {
                    GL.Vertex3(nextCurve.curList[i].easting, nextCurve.curList[i].northing, 0);
                }
                GL.End();
            }

            if (ytList2?.Count > 0)
            {
                GL.PointSize(mf.ABLine.lineWidth + 2);
                GL.Color3(0.3f, 0.941f, 0.980f);
                GL.Begin(PrimitiveType.Points);
                for (int i = 0; i < ytList2.Count; i++)
                {
                    GL.Vertex3(ytList2[i].easting, ytList2[i].northing, 0);
                }
                GL.End();
            }
        }

        public class CClose
        {
            public vec3 closePt = new vec3();
            public int turnLineNum;
            public int turnLineIndex;
            public double turnLineHeading;
            public int curveIndex;

            public CClose()
            {
                closePt = new vec3();
                turnLineNum = -1;
                turnLineIndex = -1;
                turnLineHeading = -1;
                curveIndex = -1;
            }

            public CClose(CClose _clo)
            {
                closePt = new vec3(_clo.closePt);
                turnLineNum = _clo.turnLineNum;
                turnLineIndex = _clo.turnLineIndex;
                turnLineHeading = _clo.turnLineHeading;
                curveIndex = _clo.curveIndex;
            }
        }
    }
}