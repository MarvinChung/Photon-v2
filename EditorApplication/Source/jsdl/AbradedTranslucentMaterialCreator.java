// ========================================
// NOTE: THIS FILE CONTAINS GENERATED CODE 
//       DO NOT MODIFY                     
// ========================================
// last generated: 2018-12-17 15:13:22.675086 

package jsdl;

public class AbradedTranslucentMaterialCreator extends SDLCreatorCommand
{
	@Override
	public String getFullType()
	{
		return "material(abraded-translucent)";
	}

	public void setFresnelType(SDLString data)
	{
		setInput("fresnel-type", data);
	}

	public void setRoughness(SDLReal data)
	{
		setInput("roughness", data);
	}

	public void setIorOuter(SDLReal data)
	{
		setInput("ior-outer", data);
	}

	public void setIorInner(SDLReal data)
	{
		setInput("ior-inner", data);
	}

}

