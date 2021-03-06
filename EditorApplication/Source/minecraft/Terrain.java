package minecraft;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;

import jsdl.CuboidGeometryCreator;
import jsdl.MatteOpaqueMaterialCreator;
import jsdl.ModelActorCreator;
import jsdl.SDLCommand;
import jsdl.SDLGeometry;
import jsdl.SDLMaterial;
import jsdl.SDLVector3;
import util.Vector3f;
import util.Vector3i;

public class Terrain
{
	private Map<RegionCoord, RegionData> m_regions;
	
	public Terrain()
	{
		m_regions = new HashMap<>();
	}
	
	public RegionData getRegion(int regionX, int regionZ)
	{
		return m_regions.get(new RegionCoord(regionX, regionZ));
	}
	
	public void addRegion(RegionData region)
	{
		m_regions.put(region.getRegionCoord(), region);
	}
	
	public List<SectionUnit> getAllSections()
	{
		List<SectionUnit> sections = new ArrayList<>();
		for(RegionData region : m_regions.values())
		{
			for(int chunkZ = 0; chunkZ < RegionData.NUM_CHUNKS_Z; ++chunkZ)
			{
				for(int chunkX = 0; chunkX < RegionData.NUM_CHUNKS_X; ++chunkX)
				{
					ChunkData chunk = region.getChunk(chunkX, chunkZ);
					if(chunk == null)
					{
						continue;
					}
					
					for(int s = 0; s < ChunkData.NUM_SECTIONS; ++s)
					{
						SectionData section = chunk.getSection(s);
						if(section == null)
						{
							continue;
						}
						
						final int x = region.getX() + chunkX * SectionData.SIZE_X;
						final int y = region.getY() + s * SectionData.SIZE_Y;
						final int z = region.getZ() + chunkZ * SectionData.SIZE_Z;
						sections.add(new SectionUnit(new Vector3i(x, y, z), section));
					}
				}
			}
		}
		return sections;
	}
	
	public List<SectionUnit> getReachableSections(Vector3f viewpoint)
	{
		// HACK
		int MAX_RADIUS = 8;
		
		Vector3f pv = toSectionCoord(viewpoint).toVector3f();
		System.err.println("pv: " + pv);
		
		class Flood implements Comparable<Flood>
		{
			Vector3i coord;
			EFacing  front;
			
			Flood(Vector3i coord, EFacing front)
			{
				this.coord = coord;
				this.front = front;
			}
			
			@Override
			public int compareTo(Flood other)
			{
				Vector3f p1 = coord.toVector3f();
				Vector3f p2 = other.coord.toVector3f();
				float value1 = pv.sub(p1).squareLength();
				float value2 = pv.sub(p2).squareLength();
				return value1 < value2 ? -1 : (value1 > value2 ? 1 : 0);
			}
		}
		
		Vector3i[] coordOffsets = new Vector3i[]{
			new Vector3i( 0,  0, -1),
			new Vector3i( 0,  0,  1),
			new Vector3i(-1,  0,  0),
			new Vector3i( 1,  0,  0),
			new Vector3i( 0, -1,  0),
			new Vector3i( 0,  1,  0)
		};
		
		SectionStateMap floodedArea = new SectionStateMap();
		Map<Vector3i, FaceReachability> sectionReachability = new HashMap<>();
		List<SectionUnit> reachableSections = new ArrayList<>();
		
		Map<Vector3i, SectionUnit> sectionMap = new HashMap<>();
		for(SectionUnit section : getAllSections())
		{
			sectionMap.put(Coordinate.toSection(section.getCoord()), section);
		}
		
		PriorityQueue<Flood> floodQueue = new PriorityQueue<>();
		
		{
			Vector3i rootCoord = toSectionCoord(viewpoint);
			for(EFacing front : EFacing.values())
			{
				floodQueue.add(new Flood(rootCoord, front));
			}
			floodedArea.setSection(rootCoord, true);
			
			if(sectionMap.get(rootCoord) != null)
			{
				reachableSections.add(sectionMap.get(rootCoord));
			}
		}
		
		while(!floodQueue.isEmpty())
		{
			Flood    flood = floodQueue.poll();
			Vector3i coord = flood.coord.add(coordOffsets[flood.front.getValue()]);
			if(!isInBound(coord) ||
			   coord.y < 0 || coord.y >= ChunkData.NUM_SECTIONS || 
			   floodedArea.getSection(coord))
			{
				continue;
			}
			
			if(coord.toVector3f().sub(pv).length() > MAX_RADIUS)
			{
				break;
			}
			
			SectionUnit section = sectionMap.get(coord);
			
			if(section == null)
			{
				for(EFacing nextFront : EFacing.values())
				{
					floodQueue.add(new Flood(coord, nextFront));
				}
				floodedArea.setSection(coord, true);
			}
			else
			{
				FaceReachability reachability = sectionReachability.get(coord);
				if(reachability == null)
				{
					reachability = section.getData().determinReachability();
					sectionReachability.put(coord, reachability);
					reachableSections.add(section);
				}
				
				EFacing from = flood.front.getOpposite();
				for(EFacing to : EFacing.values())
				{
					if(from != to && reachability.isReachable(from, to))
					{
						floodQueue.add(new Flood(coord, to));
						reachability.setReachability(from, to, false);
					}
				}
				
				// it is possible for flood to flow within the same section
				if(!floodedArea.getSection(flood.coord))
				{
					FaceReachability fromReachability = sectionReachability.get(flood.coord);
					if(fromReachability.isReachable(flood.front))
					{
						floodQueue.add(new Flood(coord, from));
					}
				}
				
				if(reachability.isFullyUnreachable())
				{
					floodedArea.setSection(coord, true);
				}
			}
		}// end while
		
		return reachableSections;
	}
	
	private static Vector3i toSectionCoord(Vector3f viewpoint)
	{
		float clampedY = Math.max(0.0f, Math.min(viewpoint.y, ChunkData.SIZE_Y));
		
		return new Vector3i(
			Math.floorDiv((int)Math.floor(viewpoint.x), SectionData.SIZE_X),
			Math.min((int)clampedY / SectionData.SIZE_Y, ChunkData.NUM_SECTIONS - 1),
			Math.floorDiv((int)Math.floor(viewpoint.z), SectionData.SIZE_Z));
	}
	
	private boolean isInBound(Vector3i section)
	{
		RegionCoord region = Coordinate.sectionToRegion(section);
		return m_regions.containsKey(region);
	}
}
