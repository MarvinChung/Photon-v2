// ========================================
// NOTE: THIS FILE CONTAINS GENERATED CODE 
//       DO NOT MODIFY                     
// ========================================
// last generated: 2018-12-30 09:20:21.381536 

package jsdl;

public class RectangleLightSourceCreator extends SDLCreatorCommand
{
	@Override
	public String getFullType()
	{
		return "light-source(rectangle)";
	}

	public void setWidth(SDLReal data)
	{
		setInput("width", data);
	}

	public void setHeight(SDLReal data)
	{
		setInput("height", data);
	}

	public void setLinearSrgb(SDLVector3 data)
	{
		setInput("linear-srgb", data);
	}

	public void setWatts(SDLReal data)
	{
		setInput("watts", data);
	}

}

