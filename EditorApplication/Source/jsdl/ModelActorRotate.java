// ========================================
// NOTE: THIS FILE CONTAINS GENERATED CODE 
//       DO NOT MODIFY                     
// ========================================
// last generated: 2018-12-17 15:13:22.650085 

package jsdl;

public class ModelActorRotate extends SDLExecutorCommand
{
	@Override
	public String getFullType()
	{
		return "actor(model)";
	}

	@Override
	public String getName()
	{
		return "rotate";
	}

	public void setAxis(SDLVector3 data)
	{
		setInput("axis", data);
	}

	public void setDegree(SDLReal data)
	{
		setInput("degree", data);
	}

	public void setFactor(SDLQuaternion data)
	{
		setInput("factor", data);
	}

}

