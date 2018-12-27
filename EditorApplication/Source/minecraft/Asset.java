package minecraft;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import javax.imageio.ImageIO;

import minecraft.block.BlockData;
import minecraft.parser.BlockParser;
import minecraft.parser.ModelParser;

public class Asset
{
	private Map<String, ModelData>     m_models;
	private Map<String, BufferedImage> m_textures;
	private Map<String, BlockData>     m_blocks;
	
	public Asset()
	{
		m_models   = new HashMap<>();
		m_textures = new HashMap<>();
		m_blocks   = new HashMap<>();
	}
	
	public void add(String modelId, ModelData model)
	{
		System.err.println("adding model " + model);
		
		if(model != null)
		{
			m_models.put(modelId, model);
		}
	}
	
	public void add(String textureId, BufferedImage image)
	{
		System.err.println("adding texture " + textureId);
		
		if(image != null)
		{
			m_textures.put(textureId, image);
		}
	}
	
	public void add(String blockId, BlockData block)
	{
		System.err.println("adding block " + block);
		
		if(block != null)
		{
			m_blocks.put(blockId, block);
		}
	}
	
	public void loadModels(Path modelFolder, Set<String> modelIds)
	{
		ModelParser parser = new ModelParser();
		for(String modelId : modelIds)
		{
			loadModel(modelFolder, modelId, parser);
			
			System.err.println(m_models.get(modelId).getRequiredTextures());
		}
	}
	
	public void loadTextures(Path textureFolder, Set<String> textureIds)
	{
		for(String textureId : textureIds)
		{
			if(m_textures.containsKey(textureId))
			{
				continue;
			}
			
			Path texturePath = textureFolder.resolve(textureId + ".png");
			try
			{
				byte[] textureData = Files.readAllBytes(texturePath);
				ByteArrayInputStream rawData = new ByteArrayInputStream(textureData);
				BufferedImage texture = ImageIO.read(rawData);
				add(textureId, texture);
			}
			catch(IOException e)
			{
				System.err.println("error loading texture " + texturePath);
				e.printStackTrace();
			}
		}
	}
	
	public void loadBlocks(Path blockFolder, Set<String> blockIds)
	{
		BlockParser parser = new BlockParser();
		for(String blockId : blockIds)
		{
			if(m_blocks.containsKey(blockId))
			{
				continue;
			}
			
			// block ID is in the format <namespace>:<actual ID>, we need to 
			// remove the namespace for path resolving
			String actualId = blockId.substring(blockId.indexOf(':') + 1);
			Path blockPath = blockFolder.resolve(actualId + ".json");
			
			try
			{
				byte[] blockData = Files.readAllBytes(blockPath);
				ByteArrayInputStream rawData = new ByteArrayInputStream(blockData);
				BlockData block = parser.parse(rawData);
				add(blockId, block);
			}
			catch(IOException e)
			{
				System.err.println("error loading block " + blockPath);
				e.printStackTrace();
			}
		}
	}
	
	public void loadAllIdentified(Path modelFolder, Path textureFolder)
	{
		Set<String> requiredModels = new HashSet<>();
		for(BlockData block : m_blocks.values())
		{
			requiredModels.addAll(block.getRequiredModels());
		}
		loadModels(modelFolder, requiredModels);
		
		Set<String> requiredTextures = new HashSet<>();
		for(String modelId : requiredModels)
		{
			ModelData model = m_models.get(modelId);
			requiredTextures.addAll(model.getRequiredTextures());
		}
		loadTextures(textureFolder, requiredTextures);
	}
	
	private void loadModel(Path modelFolder, String modelId, ModelParser parser)
	{
		if(m_models.containsKey(modelId))
		{
			return;
		}
		
		Path modelPath = modelFolder.resolve(modelId + ".json");
		
		ModelData model = null;
		try
		{
			byte[] modelData = Files.readAllBytes(modelPath);
			ByteArrayInputStream rawData = new ByteArrayInputStream(modelData);
			model = parser.parse(rawData);
			add(modelId, model);
		}
		catch(IOException e)
		{
			System.err.println("error loading model " + modelPath);
			e.printStackTrace();
		}
		
		if(model != null && model.hasParent())
		{
			String parentModelId = model.getParentId();
			loadModel(modelFolder, parentModelId, parser);
			model.setParent(m_models.get(parentModelId));
		}
	}
}
