//
//  dynamicChainNode.h
//  dynamicChainNode
//
//  Created by wjj on 13-11-18.
//  Copyright (c) 2013 zerowjj. All rights reserved.
//

#ifndef dynamicChainNode_dynamicChainNode_h
#define dynamicChainNode_dynamicChainNode_h

#include <maya/MPxNode.h>
#include <maya/MDataBlock.h>
#include <maya/MStatus.h>
#include <maya/MPlug.h>
#include <maya/MTypeId.h>
#include <maya/MObject.h>
#include <maya/MVector.h>

class dynamicChainNode: public MPxNode
{
public:
    dynamicChainNode();
    virtual ~dynamicChainNode();
    
    static void* creator();
    static MStatus initialize();
    
    virtual MStatus compute(const MPlug &plug, MDataBlock &dataBlock);
    MVector rotateAroundAxis(const MVector &inVector, const MVector &axis, float theta);
	float _getMin(const float a, const float b);
	float _getMax(const float a, const float b);
	float _getRandom(const int num, const float b);
public:
    static MTypeId id;
    
    static MObject inputCurve;  // nurbs curve input
    static MObject inputMesh;  // single knot mesh input

    static MObject connectPara;  // 80% of the whole mesh length, so as to form a buckle
	static MObject rotateFollow;  // contorl whether the first knot follow the curve tangent
	static MObject randomOff;  // each knot rotate randomly
	static MObject rotateOff;  // global rotate offset
	static MObject fixedType;  // fix or changeable knot number
	static MObject fixedNumber;  // knot number
	static MObject objScale;  // each knot random scale
	    
    static MObject outputMesh;  // output chain mesh
	static MObject currentNumber;  // output current knot number
};

#endif
