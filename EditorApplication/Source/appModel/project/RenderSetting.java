package appModel.project;

import appModel.GeneralOption;
import appModel.SettingGroup;

public final class RenderSetting extends SettingGroup
{
	public static final String SCENE_FILE_PATH    = "scene-file-path";
	public static final String NUM_RENDER_THREADS = "num-render-threads";
	
	private GeneralOption m_generalOption;
	
	public RenderSetting(GeneralOption generalOption)
	{
		super();
		
		m_generalOption = generalOption;
	}
	
	@Override
	public void setToDefaults()
	{
		set(SCENE_FILE_PATH,    m_generalOption.get(GeneralOption.DEFAULT_SCENE_FILE_PATH));
		set(NUM_RENDER_THREADS, "4");
	}
}
