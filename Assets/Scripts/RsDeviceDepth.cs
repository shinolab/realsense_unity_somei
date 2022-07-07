using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RsDeviceDepth : RsDevice
{
    void Update()
    {
        if (!Streaming)
            return;
        if (processMode != ProcessMode.UnityThread)
            return;
        if (m_pipeline.PollForFrames(out var frames))
        {
            Debug.Log(frames.DepthFrame.GetDistance(320, 240));
        }
    }

    protected override void WaitForFrames()
    {
        while (!stopEvent.WaitOne(0))
        {
            using (var frames = m_pipeline.WaitForFrames())
                Debug.Log(frames.DepthFrame.GetDistance(320, 240));
        }
    }
}
