// ========================================
// NOTE: THIS FILE CONTAINS GENERATED CODE 
//       DO NOT MODIFY                     
// ========================================
// last generated: 2018-12-17 15:13:22.684087 

package jsdl;

public class MatteOpaqueMaterialCreator extends SDLCreatorCommand
{
	@Override
	public String getFullType()
	{
		return "material(matte-opaque)";
	}

	public void setAlbedo(SDLReal data)
	{
		setInput("albedo", data);
	}

	public void setAlbedo(SDLVector3 data)
	{
		setInput("albedo", data);
	}

	public void setAlbedo(SDLImage data)
	{
		setInput("albedo", data);
	}

}

