package appModel;

import java.util.HashMap;
import java.util.Map;

import appModel.console.Console;
import appModel.project.Project;
import photonApi.Ph;

public final class EditorApp extends ManageableResource
{
	private static final Console CONSOLE = new Console(100);
	
	private Map<String, Project> m_projects;
	private GeneralOption        m_generalOption;
	
	public EditorApp()
	{
		super();
		
		m_projects      = new HashMap<>();
		m_generalOption = new GeneralOption();
	}
	
	@Override
	protected void initResource()
	{
		if(Ph.phInit())
			System.out.println("Photon initialized");
		else
			System.err.println("Photon initializing failed");
		
		m_generalOption.load();
	}
	
	@Override
	protected void freeResource()
	{
		if(Ph.phExit())
			System.out.println("Photon exited");
		else
			System.err.println("Photon exiting failed");
	}
	
	public Project getProject(String projectName)
	{
		return m_projects.get(projectName);
	}
	
	public Project createProject(String projectName)
	{
		if(m_projects.get(projectName) != null)
		{
			System.err.println("project already exists");
			return null;
		}
		else
		{
			Project project = new Project(projectName, this);
			project.create();
			m_projects.put(projectName, project);
			return project;
		}
	}
	
	public void deleteProject(String projectName)
	{
		Project project = m_projects.get(projectName);
		if(project == null)
		{
			System.err.println("project does not exist");
		}
		else
		{
			project.decompose();
			m_projects.remove(projectName, project);
		}
	}
	
	public GeneralOption getGeneralOption() { return m_generalOption; }
	
	public static Console getConsole() { return CONSOLE; }
	
	public static void printToConsole(String message)
	{
		CONSOLE.writeMessage(message);
	}
}
