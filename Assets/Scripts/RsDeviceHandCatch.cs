using System;
using System.Threading;
using UnityEngine;
using Intel.RealSense;
using System.Collections;
using System.Linq;

using UnityEngine.UI;
using System.Collections.Generic;
using UnityEngine.Assertions;


/// <summary>
/// Manages streaming using a RealSense Device
/// </summary>
[HelpURL("https://github.com/IntelRealSense/librealsense/tree/master/wrappers/unity")]
public class RsDeviceHandCatch : RsFrameProvider
{
    /// <summary>
    /// The parallelism mode of the module
    /// </summary>
    public enum ProcessMode
    {
        Multithread,
        UnityThread,
    }

    // public static RsDevice Instance { get; private set; }

    /// <summary>
    /// Threading mode of operation, Multithread or UnityThread
    /// </summary>
    [Tooltip("Threading mode of operation, Multithreads or Unitythread")]
    public ProcessMode processMode;

    // public bool Streaming { get; private set; }

    /// <summary>
    /// Notifies upon streaming start
    /// </summary>
    public override event Action<PipelineProfile> OnStart;

    /// <summary>
    /// Notifies when streaming has stopped
    /// </summary>
    public override event Action OnStop;

    /// <summary>
    /// Fired when a new frame is available
    /// </summary>
    public override event Action<Frame> OnNewSample;

    /// <summary>
    /// User configuration
    /// </summary>
    public RsConfiguration DeviceConfiguration = new RsConfiguration
    {
        mode = RsConfiguration.Mode.Live,
        RequestedSerialNumber = string.Empty,
        Profiles = new RsVideoStreamRequest[] {
            new RsVideoStreamRequest {Stream = Stream.Depth, StreamIndex = -1, Width = 640, Height = 480, Format = Format.Z16 , Framerate = 30 },
            new RsVideoStreamRequest {Stream = Stream.Infrared, StreamIndex = -1, Width = 640, Height = 480, Format = Format.Y8 , Framerate = 30 },
            new RsVideoStreamRequest {Stream = Stream.Color, StreamIndex = -1, Width = 640, Height = 480, Format = Format.Rgb8 , Framerate = 30 }
        }
    };

    private Thread worker;
    //private readonly AutoResetEvent stopEvent = new AutoResetEvent(false);
    protected readonly AutoResetEvent stopEvent = new AutoResetEvent(false);
    //private Pipeline m_pipeline;
    protected Pipeline m_pipeline;   //private -> protected



    /// <summary>
    /// Variable by myself
    /// </summary>

    //Self-made variables
    //public Text textDepth;
    static int searchSize = 96;
    float[] arrayDepth = new float[searchSize];

    List<int> listXPos= new List<int>();

    //Pixel to Point
    //[SerializeField] RsFrameProvider _source = null;
    private Intrinsics _intrinsics;

    Vector3 fingerPosition = Vector3.zero;

    void OnEnable()
    {
        m_pipeline = new Pipeline();

        using (var cfg = DeviceConfiguration.ToPipelineConfig())
            ActiveProfile = m_pipeline.Start(cfg);

        DeviceConfiguration.Profiles = ActiveProfile.Streams.Select(RsVideoStreamRequest.FromProfile).ToArray();

        if (processMode == ProcessMode.Multithread)
        {
            stopEvent.Reset();
            worker = new Thread(WaitForFrames);
            worker.IsBackground = true;
            worker.Start();
        }

        StartCoroutine(WaitAndStart());
    }

    private void Start()
    {
        using (var profile = this.ActiveProfile.GetStream<VideoStreamProfile>(Stream.Depth))
        {
            _intrinsics = profile.GetIntrinsics();
        };
    }

    IEnumerator WaitAndStart()
    {
        yield return new WaitForEndOfFrame();
        Streaming = true;
        if (OnStart != null)
            OnStart(ActiveProfile);
    }

    void OnDisable()
    {
        OnNewSample = null;
        // OnNewSampleSet = null;

        if (worker != null)
        {
            stopEvent.Set();
            worker.Join();
        }

        if (Streaming && OnStop != null)
            OnStop();

        if (ActiveProfile != null)
        {
            ActiveProfile.Dispose();
            ActiveProfile = null;
        }

        if (m_pipeline != null)
        {
            // if (Streaming)
            // m_pipeline.Stop();
            m_pipeline.Dispose();
            m_pipeline = null;
        }

        Streaming = false;
    }

    void OnDestroy()
    {
        // OnStart = null;
        OnStop = null;

        if (ActiveProfile != null)
        {
            ActiveProfile.Dispose();
            ActiveProfile = null;
        }

        if (m_pipeline != null)
        {
            m_pipeline.Dispose();
            m_pipeline = null;
        }
    }

    private void RaiseSampleEvent(Frame frame)
    {
        var onNewSample = OnNewSample;
        if (onNewSample != null)
        {
            onNewSample(frame);
        }
    }

    /// <summary>
    /// Worker Thread for multithreaded operations
    /// </summary>
    //private void WaitForFrames()
    protected virtual void WaitForFrames()
    {
        while (!stopEvent.WaitOne(0))
        {
            using (var frames = m_pipeline.WaitForFrames())
            {
                RaiseSampleEvent(frames);

                //search pixel in x-range(272-368) and get depth
                for(var i = 0; i < searchSize; i++)
                {
                    var pixelX = 320 - searchSize / 2 + (i + 1);
                    arrayDepth[i] = frames.DepthFrame.GetDistance(pixelX, 240);
                }

                //Get the x-pixel which has the minumum Depth
                float minDepth = arrayDepth.Where(x=>x>0).Min();
                listXPos.Clear();
                for (var i = 0; i < searchSize; i++)
                {
                    if (arrayDepth[i] == minDepth)
                    {
                        listXPos.Add(i);
                    }
                }
                var aveIndex = (listXPos.Min() + listXPos.Max()) / 2;
                var avePixelX= 320 - searchSize / 2 + (aveIndex + 1);
                //Debug.Log("avePixelX: " + avePixelX + "Depth: " + minDepth.ToString("F4") + "m");

                //Pixel to Point
                var pixel = new Vector2(avePixelX,240);
                fingerPosition= Rs2_Deproject_Pixel_to_Point (in _intrinsics, in pixel, minDepth);
                //Debug.Log("PositionX: " + fingerPosition.x + "PositionY: " + fingerPosition.y + "PositionZ: " + fingerPosition.z);
                Debug.Log("PositionX: " + fingerPosition.x);
            }
        }
    }

    public Vector3 Rs2_Deproject_Pixel_to_Point(in Intrinsics intrin, in Vector2 pixel, float depth)
    {
        Assert.IsTrue(intrin.model != Distortion.ModifiedBrownConrady);
        Assert.IsTrue(intrin.model != Distortion.Ftheta);

        float x = (pixel.x - intrin.ppx) / intrin.fx;
        float y = (pixel.y - intrin.ppy) / intrin.fy;

        if (intrin.model == Distortion.ModifiedBrownConrady)
        {
            float r2 = x * x + y * y;
            float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;
            float ux = x * f + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
            float uy = y * f + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);
            x = ux;
            y = uy;
        }

        return new Vector3(depth * x, depth * y, depth);
    }

    void Update()
    {

        //Debug.Log("Start");
        //textDepth.text = "Depth: " + depth.ToString("F4") + "m";


        if (!Streaming)
            return;

        if (processMode != ProcessMode.UnityThread)
            return;

        FrameSet frames;
        if (m_pipeline.PollForFrames(out frames))
        {
            using (frames)
                RaiseSampleEvent(frames);

        }


        //Debug.Log("Last");
    }

}
