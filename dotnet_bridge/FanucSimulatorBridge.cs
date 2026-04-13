using GB_Simulator;
using GB_Simulator.Common;
using GB_Simulator.Robots.Fanuc;
using OpenCLKernel;
using ParallelFlow.Kernel;
using ParallelFlow.Types;

namespace GBRobotics.PythonBridge;

public sealed class FanucSimulatorBridge : IDisposable
{
    private readonly OpenCl_Compute _compute;
    private readonly Controller<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d> _controller;
    private readonly Simulator<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d> _simulator;
    private bool _disposed;

    public FanucSimulatorBridge(string robotKind = "lrmate200id7l", double[][]? userFrames = null, double[][]? toolFrames = null, int platformId = 0, int deviceId = 0)
    {
        //ParallelFlow.Kernel.Config.PlatformId = platformId;
        //ParallelFlow.Kernel.Config.DeviceId = deviceId;
        var compiler = new OpenCL_Compiler();
        Console.WriteLine("Compiler created successfully");
        _compute = new OpenCl_Compute(compiler);
        Console.WriteLine("Compute created successfully");
        Compute.Instance = _compute;
        Console.WriteLine("Compute instance set successfully");
        _controller = new Controller<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d>();
        Console.WriteLine("Building model for robot kind: " + robotKind);
        _controller.AddRobot(BuildModel(robotKind));
        
        
        
        
        Console.WriteLine("Model built successfully");
        // adding the user frames and tool frames to the simulator
        
        if (userFrames is not null)
        {
            int i = 0;
            foreach (var frame in userFrames)
            {
                var pose = PoseExtensions.FromXYZWPR<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d>(frame);
                _controller.AddUserFrame(new UserFrame<double, Vector4_d, Quaternion_d, DualQuaternion_d>($"uframe_{i++}", pose.Pose.AsDualQuaternion<double, Vector4_d, Quaternion_d,Vector8_d, DualQuaternion_d>()));
            }
        }
        if (toolFrames is not null)
        {
            int i = 0;
            foreach (var frame in toolFrames)
            {
                var pose = PoseExtensions.FromXYZWPR<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d>(frame);
                _controller.AddTool(new Tool<double, Vector4_d, Quaternion_d, DualQuaternion_d>($"tframe_{i++}", pose.Pose.AsDualQuaternion<double, Vector4_d, Quaternion_d,Vector8_d, DualQuaternion_d>()));
            }
        }
        
        
        _simulator = new Simulator<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d>(_compute, _controller);        
        Console.WriteLine("Simulator created successfully");
        
    }

    /// <summary>
    /// Batched forward kinematics on the GPU via <c>Simulator.FK(POS[] ...)</c>.
    /// Each row is one pose: joint values in <b>degrees</b> (same as <see cref="Fk"/>), 6 or 7 values per row.
    /// </summary>
    public FkBatchResult FkBatch(
        double[][] jointRowsDegrees,
        int robotId = 0,
        int toolId = -1,
        double[]? robotBaseXyzwprMillimetresDegrees = null)
    {
        if (jointRowsDegrees is null || jointRowsDegrees.Length == 0)
        {
            throw new ArgumentException("At least one joint row is required.", nameof(jointRowsDegrees));
        }

        Pose_d? robotBasePose = null;
        if (robotBaseXyzwprMillimetresDegrees is not null)
        {
            if (robotBaseXyzwprMillimetresDegrees.Length != 6)
            {
                throw new ArgumentException(
                    "robotBaseXyzwprMillimetresDegrees must have exactly 6 values (X,Y,Z mm; W,P,R deg).",
                    nameof(robotBaseXyzwprMillimetresDegrees));
            }

            robotBasePose = PoseExtensions.FromXYZWPR<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d>(
                robotBaseXyzwprMillimetresDegrees);
        }

        int poseCount = jointRowsDegrees.Length;
        var poses = new Pose_d[poseCount];
        for (int i = 0; i < poseCount; i++)
        {
            var row = jointRowsDegrees[i];
            if (row is null)
            {
                throw new ArgumentException($"Joint row at index {i} is null.", nameof(jointRowsDegrees));
            }

            var rad = new double[row.Length];
            for (int j = 0; j < row.Length; j++)
            {
                rad[j] = row[j] * Math.PI / 180.0;
            }

            var full = new double[7];
            for (int j = 0; j < Math.Min(7, rad.Length); j++)
            {
                full[j] = rad[j];
            }

            poses[i] = new Pose_d(full);
        }

        _simulator.FK(
            poses,
            out bool[][] outOfRangeJointPose,
            out Matrix4x4_d[][] linkTransformPerLink,
            out Matrix4x4_d[]? toolTransforms,
            robotId,
            toolId,
            robotBasePose);

        int nLinks = linkTransformPerLink.Length;
        var linkPerPose = new double[poseCount][][];
        for (int p = 0; p < poseCount; p++)
        {
            linkPerPose[p] = new double[nLinks][];
            for (int L = 0; L < nLinks; L++)
            {
                linkPerPose[p][L] = MatrixToArray(linkTransformPerLink[L][p]);
            }
        }

        double[][]? toolPerPose = null;
        if (toolTransforms is not null)
        {
            toolPerPose = new double[poseCount][];
            for (int p = 0; p < poseCount; p++)
            {
                toolPerPose[p] = MatrixToArray(toolTransforms[p]);
            }
        }

        int nJoints = outOfRangeJointPose.Length;
        var outPerPose = new bool[poseCount][];
        for (int p = 0; p < poseCount; p++)
        {
            outPerPose[p] = new bool[nJoints];
            for (int j = 0; j < nJoints; j++)
            {
                outPerPose[p][j] = outOfRangeJointPose[j][p];
            }
        }

        return new FkBatchResult(linkPerPose, toolPerPose, outPerPose);
    }

    /// <summary>
    /// Single-pose FK; implemented via <see cref="FkBatch"/> (GPU path when batch size is 1).
    /// </summary>
    public FkResult Fk(double[] jointValues, int robotId = 0, int toolId = -1, double[]? robotBaseXyzwprMillimetresDegrees = null)
    {
        if (jointValues is null)
        {
            throw new ArgumentNullException(nameof(jointValues));
        }

        var row = new double[jointValues.Length];
        Array.Copy(jointValues, row, jointValues.Length);
        var batch = FkBatch(new[] { row }, robotId, toolId, robotBaseXyzwprMillimetresDegrees);
        return new FkResult(batch.LinkTransformsPerPose[0], null, batch.OutOfRangePerPose[0]);
    }

    public IkResult Ik(double[][] endEffectorTransforms, int robotId = 0)
    {
        if (endEffectorTransforms is null || endEffectorTransforms.Length == 0)
        {
            throw new ArgumentException("At least one end-effector transform is required.", nameof(endEffectorTransforms));
        }

        var targetPoses = endEffectorTransforms.Select(TransformArrayToPose).ToArray();
        _simulator.IK(targetPoses, out Pose_d[] solutions, out bool[] isValid, robotId, null);

        var joints = solutions.Select(PoseToJointArray).ToArray();

        return new IkResult(joints, isValid);
    }

    private static Model<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d> BuildModel(string robotKind)
    {
        var normalized = (robotKind ?? string.Empty).Trim().ToLowerInvariant();

        return normalized switch
        {
            "lrmate200id7l" or "fanuc_lrmate200id7l" =>
                new fanuc_lrmate_model<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d>(
                    new Fanuc_lrmate200id7l_robot<double, Vector4_d, Quaternion_d, DualQuaternion_d>(),
                    "fanuc_lrmate200id7l_model"
                ),
            "lr10ia10" or "fanuc_lr_10ia_10" =>
                new fanuc_lrmate_model<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d, Pose_d, Matrix4x4_d>(
                    new fanuc_lr_10iA_10_robot<double, Vector4_d, Quaternion_d, DualQuaternion_d>(),
                    "fanuc_lr_10ia_10_model"
                ),
            _ => throw new ArgumentException($"Unknown robot kind '{robotKind}'.")
        };
    }

    private static Pose_d TransformArrayToPose(double[] transform)
    {
        var matrix = ArrayToMatrix(transform);
        var dq = matrix.ToDualQuaternion<double, Vector4_d, Quaternion_d, DualQuaternion_d, Matrix4x4_d>();
        var dv = dq.AsDualVector<double, Vector4_d, Quaternion_d, Vector8_d, DualQuaternion_d>();

        return new Pose_d
        {
            Pose = dv,
            ToolId = 0,
            FrameId = 0
        };
    }

    private static double[] PoseToJointArray(Pose_d pose)
    {
        return new[]
        {
            pose.Pose.V1.X,
            pose.Pose.V1.Y,
            pose.Pose.V1.Z,
            pose.Pose.V2.X,
            pose.Pose.V2.Y,
            pose.Pose.V2.Z,
            pose.E1
        };
    }

    private static Matrix4x4_d ArrayToMatrix(double[] values)
    {
        if (values is null || values.Length != 16)
        {
            throw new ArgumentException("Transform matrix must have exactly 16 values.", nameof(values));
        }

        return new Matrix4x4_d
        {
            Row1 = new Vector4_d(values[0], values[1], values[2], values[3]),
            Row2 = new Vector4_d(values[4], values[5], values[6], values[7]),
            Row3 = new Vector4_d(values[8], values[9], values[10], values[11]),
            Row4 = new Vector4_d(values[12], values[13], values[14], values[15])
        };
    }

    private static double[] MatrixToArray(Matrix4x4_d matrix)
    {
        return new[]
        {
            matrix.Row1.X, matrix.Row1.Y, matrix.Row1.Z, matrix.Row1.W,
            matrix.Row2.X, matrix.Row2.Y, matrix.Row2.Z, matrix.Row2.W,
            matrix.Row3.X, matrix.Row3.Y, matrix.Row3.Z, matrix.Row3.W,
            matrix.Row4.X, matrix.Row4.Y, matrix.Row4.Z, matrix.Row4.W
        };
    }



    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        if (_compute is not null)
        {
            var message = string.Empty;
            _compute.CleanUp(ref message, true);
        }

        _disposed = true;
        GC.SuppressFinalize(this);
    }

    ~FanucSimulatorBridge()
    {
        Dispose();
    }

    #region static methods for value conversions
    
    #endregion
}

public sealed class FkResult
{
    public FkResult(double[][] linkTransforms, double[][]? toolTransforms, bool[] outOfRange)
    {
        LinkTransforms = linkTransforms;
        ToolTransforms = toolTransforms;
        OutOfRange = outOfRange;
    }

    public double[][] LinkTransforms { get; }
    public double[][]? ToolTransforms { get; }
    public bool[] OutOfRange { get; }
}

/// <summary>
/// One GPU batched FK call: <see cref="FanucSimulatorBridge.FkBatch"/>.
/// </summary>
public sealed class FkBatchResult
{
    public FkBatchResult(double[][][] linkTransformsPerPose, double[][]? toolTransformsPerPose, bool[][] outOfRangePerPose)
    {
        LinkTransformsPerPose = linkTransformsPerPose;
        ToolTransformsPerPose = toolTransformsPerPose;
        OutOfRangePerPose = outOfRangePerPose;
    }

    /// <summary>Shape [pose][link][16] row-major 4×4.</summary>
    public double[][][] LinkTransformsPerPose { get; }

    /// <summary>Shape [pose][16] when tool_id ≥ 0; otherwise null.</summary>
    public double[][]? ToolTransformsPerPose { get; }

    /// <summary>Shape [pose][joint].</summary>
    public bool[][] OutOfRangePerPose { get; }
}

public sealed class IkResult
{
    public IkResult(double[][] joints, bool[] isValid)
    {
        Joints = joints;
        IsValid = isValid;
    }

    public double[][] Joints { get; }
    public bool[] IsValid { get; }
}
