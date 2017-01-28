package core;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import photonApi.FloatArrayRef;
import photonApi.FloatRef;
import photonApi.IntRef;
import photonApi.LongRef;
import photonApi.Ph;
import photonApi.PhTest;
import photonCore.FrameData;
import photonCore.PhDescription;
import photonCore.PhFrame;
import photonCore.PhRenderer;
import ui.Window;
import ui.display.ImagePanel;
import ui.model.TaskStatusModel;

public class Main
{
	private static Window window;
	
	public static void main(String[] args)
	{
		if(!Ph.phStart())
		{
			System.out.println("Photon API initialization failed");
		}
		
		new PhTest();
		
		TaskStatusModel testTaskStatusModel = new TaskStatusModel();
		TaskStatusModel dummyTaskStatusModel = new TaskStatusModel();
		
		try
		{
			SwingUtilities.invokeAndWait(new Runnable()
			{
				@Override
				public void run()
				{
					window = new Window();
					window.getTaskPanel().registerTaskStatusModel(testTaskStatusModel);
					window.getTaskPanel().registerTaskStatusModel(dummyTaskStatusModel);
				}
			});
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		
		testTaskStatusModel.setTaskName("test render task");
		dummyTaskStatusModel.setTaskName("dummy dummy dummy");
		
		// for cbox
		//camera.setPosition(0.000001f, -0.000002f, 16);
//		camera.setPosition(0.000001f, -0.000002f, 3);
		//camera.setDirection(0.0001f, 0.000002f, -1.0f);
		
//		camera.setPosition(0.000001f, -0.500002f, 17);
//		camera.setDirection(0.0001f, 0.100002f, -1.0f);
		
		// for sponza
//		camera.setPosition(3.5765076f, 2.1717842f, 2.5685565f);
//		camera.setDirection(-0.81385213f, -0.30174536f, -0.49657395f);
		
		//ArrayList<PhCamera> bb;
		
//		final int outputWidth = 1280;
//		final int outputHeight = 720;
//		final int outputWidth = 1400;
//		final int outputHeight = 600;
//		final int outputWidth = 800;
//		final int outputHeight = 800;
//		final int outputWidth = 400;
//		final int outputHeight = 400;
//		final int outputWidth = 150;
//		final int outputHeight = 150;
		final int numRenderThreads = 4;
		
		
		PhDescription description = new PhDescription();
//		description.load("../scene/testScene.p2");
		description.load("../scene/cornell_box.p2");
		description.update();
		
		PhRenderer renderer = new PhRenderer(numRenderThreads);
		
		Thread queryThread = new Thread((new Runnable()
		{
			@Override
			public void run()
			{
				while(true)
				{
					float progress = renderer.queryPercentageProgress();
					float frequency = renderer.querySampleFrequency();
					testTaskStatusModel.setPercentageProgress(progress);
					testTaskStatusModel.setSampleFrequency(frequency);
					if(progress == 100.0f)
					{
						break;
					}
					
					try
					{
						Thread.sleep(3000);
					}
					catch(InterruptedException e)
					{
						e.printStackTrace();
					}
				}
				
				System.out.println("query thread end");
			}
		}));
		queryThread.start();
		
		long t1 = System.currentTimeMillis();
		renderer.render(description);
		long t2 = System.currentTimeMillis();
		System.out.println("time elapsed: " + (double)(t2 - t1) + " ms");
		
		PhFrame frame = new PhFrame(PhFrame.Type.HDR);
		description.developFilm(frame);
		
		FrameData frameData = new FrameData();
		frame.getData(frameData);
		
		System.out.println("frame width: " + frameData.getWidthPx() + " | frame height: " + frameData.getHeightPx());
		
		HdrFrame hdrFrame = new HdrFrame(frameData);
		
		SwingUtilities.invokeLater(new Runnable()
		{
			@Override
			public void run()
			{
				window.getDisplayPanel().render(hdrFrame);
				System.out.println("rendering done");
			}
		});
		
		// TODO: exit photon
	}
}
