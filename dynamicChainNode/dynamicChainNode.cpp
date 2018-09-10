//
//  dynamicChainNode.cpp
//  dynamicChainNode
//
//  Created by wjj on 13-11-18.
//  Copyright (c) 2013 zerowjj. All rights reserved.
//

#include "dynamicChainNode.h"
#include <stdlib.h>
#include <math.h>

#include <maya/MPxNode.h>
#include <maya/MStatus.h>
#include <maya/MPlug.h>
#include <maya/MTypeId.h>
#include <maya/MObject.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnNurbsCurveData.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnPlugin.h>
#include <maya/MDataHandle.h>
#include <maya/MFnData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnDagNode.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MPxTransformationMatrix.h>
#include <maya/MPxTransform.h>
#include <maya/MEulerRotation.h>
#include <maya/MPxConstraint.h>
#include <maya/MMatrixArray.h>
#include <maya/MStringArray.h>
#include <maya/MFloatArray.h>
#include <maya/MPxCommand.h>
#include <maya/MString.h>

MTypeId dynamicChainNode::id(0x0FEA1);
MObject dynamicChainNode::inputCurve;
MObject dynamicChainNode::inputMesh;

MObject dynamicChainNode::connectPara;
MObject dynamicChainNode::rotateFollow;
MObject dynamicChainNode::randomOff;
MObject dynamicChainNode::rotateOff;
MObject dynamicChainNode::fixedType;
MObject dynamicChainNode::fixedNumber;
MObject dynamicChainNode::objScale;

MObject dynamicChainNode::outputMesh;
MObject dynamicChainNode::currentNumber;

dynamicChainNode::dynamicChainNode()
{}

dynamicChainNode::~dynamicChainNode()
{}

void* dynamicChainNode::creator()
{
    return new dynamicChainNode();
}

MStatus dynamicChainNode::initialize()
{
    MStatus status;
    MFnNumericAttribute numericAttr;
    MFnTypedAttribute typeAttr;
    
	inputCurve = typeAttr.create("inputCurve", "ic", MFnNurbsCurveData::kNurbsCurve);
    typeAttr.setStorable(true);
    typeAttr.setKeyable(false);
    inputMesh = typeAttr.create("inputMesh", "im", MFnMeshData::kMesh);
    typeAttr.setStorable(true);
    typeAttr.setKeyable(false);
    
    connectPara = numericAttr.create("connectParameter", "cp", MFnNumericData::kDouble);
    numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(0.2);
    numericAttr.setMin(0.0);
    numericAttr.setMax(0.9);
	rotateFollow = numericAttr.create("rotateFollow", "rf", MFnNumericData::kBoolean);
	numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(1);
	randomOff = numericAttr.create("randomOffset", "ranO", MFnNumericData::kDouble);
    numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(0.0);
    numericAttr.setMin(0.0);
    numericAttr.setMax(45.0);
	rotateOff = numericAttr.create("rotateOffset", "rotO", MFnNumericData::kDouble);
	numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(0.0);
    numericAttr.setMin(0.0);
    numericAttr.setMax(90.0);
	fixedType = numericAttr.create("fixedType", "ft", MFnNumericData::kInt);
	numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(0);
    numericAttr.setMin(0);
	numericAttr.setMax(2);
	fixedNumber	= numericAttr.create("fixedNumber", "fn", MFnNumericData::kInt);
	numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(0);
    numericAttr.setMin(0);
	objScale = numericAttr.create("objScale", "os", MFnNumericData::kDouble);
	numericAttr.setStorable(true);
    numericAttr.setKeyable(true);
    numericAttr.setDefault(1.0);
    numericAttr.setMin(0.001);

    outputMesh = typeAttr.create("outputMesh", "om", MFnMeshData::kMesh);
    typeAttr.setStorable(false);
    typeAttr.setWritable(false);
	currentNumber = numericAttr.create("currentNumber", "cn", MFnNumericData::kInt);
    numericAttr.setStorable(false);
    numericAttr.setWritable(false);

    addAttribute(connectPara);
	addAttribute(rotateFollow);
	addAttribute(randomOff);
	addAttribute(rotateOff);
	addAttribute(fixedType);
	addAttribute(fixedNumber);
	addAttribute(objScale);	

    addAttribute(inputCurve);
    addAttribute(inputMesh);

    addAttribute(outputMesh);
	addAttribute(currentNumber);
    
    attributeAffects(connectPara, outputMesh);
	attributeAffects(rotateFollow, outputMesh);
	attributeAffects(randomOff, outputMesh);
	attributeAffects(rotateOff, outputMesh);
	attributeAffects(fixedType, outputMesh);
	attributeAffects(fixedNumber, outputMesh);
	attributeAffects(objScale, outputMesh);
    attributeAffects(inputCurve, outputMesh);
    attributeAffects(inputMesh, outputMesh);

	attributeAffects(connectPara, currentNumber);
	attributeAffects(rotateFollow, currentNumber);
	attributeAffects(randomOff, currentNumber);
	attributeAffects(rotateOff, currentNumber);
	attributeAffects(fixedType, currentNumber);
	attributeAffects(fixedNumber, currentNumber);
    attributeAffects(inputCurve, currentNumber);
    attributeAffects(inputMesh, currentNumber);
    
    return MS::kSuccess;
}

MVector dynamicChainNode::rotateAroundAxis(const MVector &inVector, const MVector &axis, float theta)
{
    // compute the vector that rotate around the tangent vector
    if(theta == 0)
        return inVector;
    
    MVector inVec(inVector);
    inVec.normalize();
    
    MVector inAxis(axis);
    inAxis.normalize();
    
    MVector upVec = inVec ^ inAxis;
    upVec.normalize();
    
    MVector fntVec = inAxis * inVec * inAxis;
    MVector sideVec = inVec - fntVec;
    
    MVector sideVecX = sideVec * cos(theta);
    MVector sideVecY = upVec * (sideVec.length() * sin(theta));
    
    MVector rotVec = fntVec + sideVecX + sideVecY;
    return rotVec;
}

float dynamicChainNode::_getMin(const float a, const float b)
{
	if(a < b)
		return a;
	else
		return b;
}

float dynamicChainNode::_getMax(const float a, const float b)
{
	if(a < b)
		return b;
	else
		return a;
}

float dynamicChainNode::_getRandom(const int num, const float max)
{
	if( max == 0.0)
		return 0.0;
	else
	{
		int tem = num * num * num;
		float rev = fmod(tem, max);
		return rev;
	}
}


MStatus dynamicChainNode::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status;
    if((plug.attribute() != outputMesh) && (plug.attribute() != currentNumber))
        return status;

	// data getter
	MDataHandle inputCurveHandle = dataBlock.inputValue(inputCurve);
    MDataHandle inputMeshHandle = dataBlock.inputValue(inputMesh);

    MDataHandle connectParaHandle = dataBlock.inputValue(connectPara);
	MDataHandle rotateFollowHandle = dataBlock.inputValue(rotateFollow);
	MDataHandle randomOffHandle = dataBlock.inputValue(randomOff);
	MDataHandle rotateOffHandle = dataBlock.inputValue(rotateOff);
	MDataHandle fixedTypeHandle = dataBlock.inputValue(fixedType);
    MDataHandle fixedNumberHandle = dataBlock.inputValue(fixedNumber);
	MDataHandle objScaleHandle = dataBlock.inputValue(objScale);
	
    MDataHandle outputMeshHandle = dataBlock.outputValue(outputMesh);
	MDataHandle currentNumberHandle = dataBlock.outputValue(currentNumber);

    if(connectParaHandle.type() != MFnData::kNumeric ||
	   rotateFollowHandle.type() != MFnData::kNumeric ||
	   randomOffHandle.type() != MFnData::kNumeric ||
	   rotateOffHandle.type() != MFnData::kNumeric ||
	   fixedTypeHandle.type() != MFnData::kNumeric ||
	   fixedNumberHandle.type() != MFnData::kNumeric ||
	   objScaleHandle.type() != MFnData::kNumeric ||
	   inputCurveHandle.type() != MFnData::kNurbsCurve ||
	   inputMeshHandle.type() != MFnData::kMesh)
    {   return MS::kInvalidParameter;   }
    
    double para = connectParaHandle.asDouble();
	double ranOff = randomOffHandle.asDouble();
    double rotOff = rotateOffHandle.asDouble();
	bool rotFollow = rotateFollowHandle.asBool();
	unsigned fixTyp = fixedTypeHandle.asInt();
	unsigned fixNum	= fixedNumberHandle.asInt();
	double objSca = objScaleHandle.asDouble();

    MObject iCurve = inputCurveHandle.asNurbsCurve();
    MFnNurbsCurve inCurve;
    inCurve.setObject(iCurve);
    
    MObject iMesh = inputMeshHandle.asMesh();
    MFnMesh inMesh;
    inMesh.setObject(iMesh);
    
    // real compute method
    MBoundingBox inMeshBBox = inMesh.boundingBox();
    double curveLength = inCurve.length();
    
    int inCurveCvNum = inCurve.numCVs();
    MPoint startCvPnt;
    MPoint endCvPnt;
    double startParam;
    double endParam;
    inCurve.getCV(0, startCvPnt);
    inCurve.getParamAtPoint(startCvPnt, startParam);
    inCurve.getCV(inCurveCvNum, endCvPnt);
    inCurve.getParamAtPoint(endCvPnt, endParam);
    
    // mesh info getter
    MPointArray meshPntArray;
    inMesh.getPoints(meshPntArray);
    int meshPntNum = meshPntArray.length();
    int inPolyNum = inMesh.numPolygons();

    MIntArray inPerPolyVertexCountArray;
    MIntArray inVertexArray;
    inMesh.getVertices(inPerPolyVertexCountArray, inVertexArray);

    MStringArray uvSetName;
    inMesh.getUVSetNames(uvSetName);
    MFloatArray inUArray, inVArray;
    MIntArray inUvIds;
    MString curUvSet = uvSetName[0];
    inMesh.getUVs(inUArray, inVArray);
    inMesh.getAssignedUVs(inPerPolyVertexCountArray, inUvIds);
    unsigned inUvCount = inUArray.length();

	// compute each knot's real length: 80%(connectPara) length
    float maxY = 0.0;
    float minY = 0.0;
    for(unsigned j = 0; j < meshPntNum; j++)
    {
		maxY = _getMax(maxY, meshPntArray[j].y);
        minY = _getMin(minY, meshPntArray[j].y);
    }
    
    double meshHeight = (maxY - minY) * (1 - para);
    int objNum = ceil(curveLength / meshHeight);  // minimum knot num
	int resNum = 0;
	if(fixNum != 0)
	{
		if(fixTyp == 0)
		{
			objNum = fixNum;
			meshHeight = curveLength / objNum;
		}
		else
		{
			objNum = _getMin(fixNum, objNum);
			meshHeight = curveLength / objNum;
			resNum = _getMax((fixNum - objNum), resNum);
		}
	}
    
    // nurbs curve message getter
    MPoint lastPoint;
    MVector curUpVec(0.0, 1.0, 0.0);
    MVector lastSideVec;
        
    // main brain
    MPointArray outPntArray;
    MIntArray outPerPolyVertexCountArray;
    MIntArray outVertexArray;
    MFloatArray outUArray, outVArray;
    MIntArray outUvIds;
    unsigned outPolyCount = 0;
	MMatrix lastMatrix;
	float scaleDis = objSca - 1.0;
	float eachScale	= scaleDis / objNum;
	
	for(unsigned i = 0; i < (objNum + resNum + 1); i++)
	{
		double curLen = i * meshHeight;
		double curPara = inCurve.findParamFromLength(curLen);
		MPoint curPoint;
		inCurve.getPointAtParam(curPara, curPoint);
		if(i != 0)
		{
			double matArray[4][4] = {
				1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0};
			if(i <= objNum)
			{
				MVector toVec = curPoint - lastPoint;
				toVec.normalize();

				MPoint toPnt = lastPoint + toVec * meshHeight * 0.5;
	            
				curUpVec = toVec ^ lastSideVec;
				double randNum = _getRandom(i, ranOff);
				//curI *= -1;
				double rotVal = (90 + randNum) * 0.017453;
				MVector rotUpVec = rotateAroundAxis(curUpVec, toVec, rotVal);
				rotUpVec.normalize();

				lastSideVec = rotUpVec ^ toVec;
				lastSideVec.normalize();
	            
				float scaleFactor = 1.0 + _getMin(i, objNum) * eachScale;
				matArray[0][0] = rotUpVec.x * scaleFactor;					
				matArray[0][1] = rotUpVec.y * scaleFactor;
				matArray[0][2] = rotUpVec.z * scaleFactor;
				matArray[1][0] = toVec.x;
				matArray[1][1] = toVec.y;
				matArray[1][2] = toVec.z;
				matArray[2][0] = lastSideVec.x * scaleFactor;
				matArray[2][1] = lastSideVec.y * scaleFactor;
				matArray[2][2] = lastSideVec.z * scaleFactor;
				matArray[3][0] = toPnt.x;
				matArray[3][1] = toPnt.y;
				matArray[3][2] = toPnt.z;
			}

			MMatrix curMatrix(matArray);
			if(i == objNum && fixTyp == 1)
			{
				lastMatrix = curMatrix;
			}
						
            if(i > objNum)
			{
				curMatrix = lastMatrix;
			}	

			for(unsigned j = 0; j < meshPntNum; j++)
			{
				MPoint outMeshPnt = meshPntArray[j] * curMatrix;
				outPntArray.append(outMeshPnt);
			}
            
			for(unsigned j = 0; j < inPerPolyVertexCountArray.length(); j++)
				outPerPolyVertexCountArray.append(inPerPolyVertexCountArray[j]);
            
			for(unsigned j = 0; j < inVertexArray.length(); j++)
				outVertexArray.append(inVertexArray[j] + (i-1) * meshPntNum);
            
			for(unsigned j = 0; j < inUArray.length(); j++)
			{
				outUArray.append(inUArray[j]);
				outVArray.append(inVArray[j]);
			}
            
			for(unsigned j = 0; j < inUvIds.length(); j++)
				outUvIds.append(inUvIds[j] + (i-1) * inUvCount);
            
			outPolyCount += inPolyNum;            
		}
		else if(i == 0)
		{
			if(rotFollow == 1)
			{
				MVector startTan = inCurve.tangent(curPara, MSpace::kWorld);
				MVector newCurUpVec = rotateAroundAxis(curUpVec, startTan, (rotOff * 0.017453));
				lastSideVec = newCurUpVec ^ startTan;
			}
			else
			{
				MVector temVec(1.0, 1.0, 0.0);
				lastSideVec = temVec;
			}
			lastSideVec.normalize();
		}
        
		lastPoint = curPoint;
	}
    
    //out mesh create
    MFnMeshData outDataCreator;
    MObject outData = outDataCreator.create();
    unsigned outPntCount = outPntArray.length();
    
    MFnMesh outMesh;
    outMesh.create(outPntCount, outPolyCount, outPntArray, outPerPolyVertexCountArray, outVertexArray, outData);
    outMesh.setUVs(outUArray, outVArray);
    outMesh.assignUVs(outPerPolyVertexCountArray, outUvIds);
    outputMeshHandle.setMObject(outData);
    
	currentNumberHandle.setInt(objNum);
	dataBlock.setClean(plug);
    return MS::kSuccess;    
}

MStatus initializePlugin(MObject obj)
{
    MStatus result;
    MFnPlugin plugin(obj, "jjCG", "1.0", "Any");
    result = plugin.registerNode("dynamicChainNode", dynamicChainNode::id, dynamicChainNode::creator, dynamicChainNode::initialize, MPxNode::kDependNode);
    return result;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus result;
    MFnPlugin plugin(obj);
    result = plugin.deregisterNode(dynamicChainNode::id);
    return result;
}
