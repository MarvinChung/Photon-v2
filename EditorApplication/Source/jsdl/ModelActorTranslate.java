// ========================================
// NOTE: THIS FILE CONTAINS GENERATED CODE 
//       DO NOT MODIFY                     
// ========================================
// last generated: 2018-12-17 15:13:22.649085 

package jsdl;

public class ModelActorTranslate extends SDLExecutorCommand
{
	@Override
	public String getFullType()
	{
		return "actor(model)";
	}

	@Override
	public String getName()
	{
		return "translate";
	}

	public void setFactor(SDLVector3 data)
	{
		setInput("factor", data);
	}

}

