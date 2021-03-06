package appGui;

public final class AppMainGraphicalState
{
	private AppMainCtrl m_appMainController;
	
	private String m_activeProjectName;
	private String m_activeViewName;
	
	public AppMainGraphicalState(AppMainCtrl appMainController)
	{
		m_appMainController = appMainController;
		
		m_activeProjectName = null;
		m_activeViewName    = null;
	}
	
	public String getActiveProjectName()
	{
		return m_activeProjectName;
	}
	
	public String getActiveViewName()
	{
		return m_activeViewName;
	}
	
	public void setActiveViewName(String name)
	{
		m_activeViewName = name;
		m_appMainController.updateFooterText();
	}
	
	public void setActiveProject(String name)
	{
		m_activeProjectName = name;
		m_appMainController.updateFooterText();
	}
}
