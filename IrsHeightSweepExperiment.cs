using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// wine-cellar 환경에서 IRS 천장 높이별로 r-WINE 링크를 평가하고
/// 높이별 요약 CSV(Height, Threshold_Deg 15/20/25, Distance)를 생성하는 실험 스크립트.
/// </summary>
[ExecuteAlways]
public class IrsHeightSweepExperiment : MonoBehaviour
{
    [Header("Scene References")]
    [Tooltip("랙 루트. 이 아래 자식들에서 TransceiverChildName을 가진 자식을 찾아서 트랜시버로 사용합니다.")]
    [SerializeField] private Transform racksRoot;

    [Tooltip("패널로 취급할 자식 이름 prefix (예: \"Element_\"). 비워두면 panelsRoot의 모든 직계 자식을 패널로 사용.")]
    [SerializeField] private string panelNamePrefix = "Element_";


    [Tooltip("IRS 패널들이 직접 포함된 루트(예: Panels). 모든 직계 자식을 IRS 패널로 간주합니다.")]
    [SerializeField] private Transform panelsRoot;

    [Tooltip("IRS 전체를 움직일 루트(예: IRS_3). 높이 스윕 시 이 오브젝트의 position만 평행 이동합니다.")]
    [SerializeField] private Transform irsRoot;

    [Tooltip("IRS 보드(천장 기준점이 되는 Transform). board의 world Y를 targetBoardHeights와 맞춥니다.")]
    [SerializeField] private Transform irsBoard;

    [Tooltip("랙 하위에서 트랜시버로 사용할 child 이름.")]
    [SerializeField] private string transceiverChildName = "Transceiver";

    [Tooltip("패널 로컬 좌표계에서의 법선 벡터. 예: 로컬 -Y가 룸 내부를 향한다면 (0,-1,0).")]
    [SerializeField] private Vector3 panelLocalNormal = Vector3.down;

    [Header("Experiment Setup")]
    [SerializeField] private bool calculateOnStart = false;

    [Tooltip("true면 targetBoardHeights 배열을 순회하면서 여러 높이를 한 번에 계산합니다. false면 현재 irsBoard 높이만 평가합니다.")]
    [SerializeField] private bool sweepHeights = true;

    [Tooltip("세계 좌표계에서 IRS 보드의 목표 높이들 (m 단위, world Y). sweepHeights=true일 때만 사용.")]
    [SerializeField] private float[] targetBoardHeights;

    [Tooltip("결과 CSV를 저장할 폴더 이름 (Assets/ 아래에 생성).")]
    [SerializeField] private string outputFolderName = "IRS_HEIGHT_SWEEP";

    [Tooltip("정반사와 간주할 최대 steering 오차 각도 (deg). 이 값 이하이면 Specular, 초과면 Steering으로 라벨링.")]
    [SerializeField] private float reflectionAngleToleranceDeg = 5f;

    [Tooltip("true면 IRS_All_Paths.csv도 생성합니다. false면 최소 경로/통계만 생성.")]
    [SerializeField] private bool writeAllPathCsv = true;

    [Header("Debug")]
    [Tooltip("실행 시 콘솔에 상세 로그를 남길지 여부.")]
    [SerializeField] private bool verboseLog = true;

    // 내부 상태
    private readonly List<Transform> _transceivers = new List<Transform>();
    private readonly List<Transform> _panels = new List<Transform>();

    // (txId -> (rxId -> BestPathRecord))
    private readonly Dictionary<int, Dictionary<int, BestPathRecord>> _bestPathByTxRx =
        new Dictionary<int, Dictionary<int, BestPathRecord>>();

    private const string AllPathsHeader =
        "TxId,RxId,PanelId," +
        "TxX,TxY,TxZ,RxX,RxY,RxZ,PanelX,PanelY,PanelZ," +
        "IncidentAngleDeg,ReflectionAngleDeg,ReflectionErrorDeg,SteeringAngleDeg,SpecularStatus," +
        "DistanceTxToPanel_m,DistancePanelToRx_m,TotalDistance_m";

    private const string MinPathsHeader =
            "TxId,RxId,PanelId," +
            "TxX,TxY,TxZ,RxX,RxY,RxZ,PanelX,PanelY,PanelZ," +
            "IncidentAngleDeg,ReflectionAngleDeg,ReflectionErrorDeg,SteeringAngleDeg,SpecularStatus," +
            "DistanceTxToPanel_m,DistancePanelToRx_m,TotalDistance_m";

    private void Start()
    {
        // 에디터에서만 사용하는 경우도 있으므로, 재생 중 + 옵션 체크일 때만 자동 실행
        if (!Application.isPlaying && !calculateOnStart)
            return;

        if (calculateOnStart)
        {
            RunExperiment();
        }
    }

#if UNITY_EDITOR
    private void OnValidate()
    {
        // 인스펙터에서 값 바꿀 때 targetBoardHeights가 null인 상태에서 실수하지 않도록 보호
        if (targetBoardHeights == null)
            targetBoardHeights = Array.Empty<float>();
    }
#endif

    [ContextMenu("Run IRS Height Sweep Experiment")]
    public void RunExperiment()
    {
        if (racksRoot == null || panelsRoot == null || irsRoot == null || irsBoard == null)
        {
            Debug.LogError("[IrsHeightSweepExperiment] Scene References가 모두 지정되어 있어야 합니다.");
            return;
        }

        CollectTransceiversAndPanels();
        if (_transceivers.Count == 0)
        {
            Debug.LogError("[IrsHeightSweepExperiment] 수집된 트랜시버가 없습니다.");
            return;
        }
        if (_panels.Count == 0)
        {
            Debug.LogError("[IrsHeightSweepExperiment] 수집된 IRS 패널이 없습니다.");
            return;
        }


        string rootFolderPath = Path.Combine(Application.dataPath, outputFolderName);
        if (!Directory.Exists(rootFolderPath))
            Directory.CreateDirectory(rootFolderPath);

        Vector3 originalIrsRootPos = irsRoot.position;
        float baseBoardY = irsBoard.position.y;

        var heightSummaries = new List<HeightSummary>();

        try
        {
            if (sweepHeights && targetBoardHeights != null && targetBoardHeights.Length > 0)
            {
                foreach (float targetH in targetBoardHeights)
                {
                    float deltaY = targetH - baseBoardY;
                    irsRoot.position = originalIrsRootPos + new Vector3(0f, deltaY, 0f);

                    string tag = $"H_{targetH:F2}m".Replace('.', '_');
                    string folder = Path.Combine(rootFolderPath, tag);
                    if (!Directory.Exists(folder))
                        Directory.CreateDirectory(folder);

                    if (verboseLog)
                        Debug.Log($"[IrsHeightSweep] === Start Height {targetH:F3} m ({tag}) ===");

                    HeightSummary summary = EvaluateCurrentHeight(folder, tag, targetH);
                    heightSummaries.Add(summary);
                }
            }
            else
            {
                float currentH = irsBoard.position.y;
                string tag = $"H_{currentH:F2}m".Replace('.', '_');
                string folder = Path.Combine(rootFolderPath, tag);
                if (!Directory.Exists(folder))
                    Directory.CreateDirectory(folder);

                if (verboseLog)
                    Debug.Log($"[IrsHeightSweep] === Start Height {currentH:F3} m ({tag}) ===");

                HeightSummary summary = EvaluateCurrentHeight(folder, tag, currentH);
                heightSummaries.Add(summary);
            }

            // IRS 위치 원복
            irsRoot.position = originalIrsRootPos;

            // 최종 HeightSummary CSV 작성
            string summaryPath = Path.Combine(rootFolderPath, "IRS_HeightSummary.csv");
            WriteHeightSummaryCsv(summaryPath, heightSummaries);

            Debug.Log($"[IrsHeightSweep] Finished. Height summary written to: {summaryPath}");
        }
        catch (Exception ex)
        {
            irsRoot.position = originalIrsRootPos;
            Debug.LogError("[IrsHeightSweepExperiment] Exception during experiment.");
            Debug.LogException(ex);
        }
    }

    /// <summary>
    /// 현재 IRS 높이에 대해
    ///  - (선택) IRS_All_Paths.csv
    ///  - IRS_MinSteering_Paths.csv
    ///  - IRS_MinSteering_Stats.csv
    /// 를 생성하고, HeightSummary(임계각 비율 + 평균 거리)를 반환.
    /// </summary>
    private HeightSummary EvaluateCurrentHeight(string folderPath, string labelForLogs, float boardHeight)
    {
        string allFilePath = Path.Combine(folderPath, "IRS_All_Paths.csv");
        string minFilePath = Path.Combine(folderPath, "IRS_MinSteering_Paths.csv");
        string statsFilePath = Path.Combine(folderPath, "IRS_MinSteering_Stats.csv");

        _bestPathByTxRx.Clear();

        StreamWriter writerAll = null;
        try
        {
            if (writeAllPathCsv)
            {
                writerAll = new StreamWriter(allFilePath, false, Encoding.UTF8);
                writerAll.WriteLine(AllPathsHeader);
            }

            // ---------- 1) 모든 (Tx,Rx,Panel) 경로 평가 + 최소 steering 경로 갱신 ----------
            int panelCount = _panels.Count;
            int txCount = _transceivers.Count;
            int rxCount = _transceivers.Count;

            for (int txIndex = 0; txIndex < txCount; txIndex++)
            {
                Transform tx = _transceivers[txIndex];
                int txId = txIndex + 1;

                for (int rxIndex = 0; rxIndex < rxCount; rxIndex++)
                {
                    if (rxIndex == txIndex)
                        continue; // 자기 자신으로 가는 링크는 스킵

                    Transform rx = _transceivers[rxIndex];
                    int rxId = rxIndex + 1;

                    for (int panelIndex = 0; panelIndex < panelCount; panelIndex++)
                    {
                        Transform panel = _panels[panelIndex];
                        int panelId = panelIndex + 1;

                        BestPathRecord record = EvaluateSinglePath(tx, rx, panel, txId, rxId, panelId);

                        if (writeAllPathCsv && writerAll != null)
                        {
                            writerAll.WriteLine(
                                $"{record.txRackId},{record.rxRackId},{record.irsId}," +
                                $"{record.txPosX:F3},{record.txPosY:F3},{record.txPosZ:F3}," +
                                $"{record.rxPosX:F3},{record.rxPosY:F3},{record.rxPosZ:F3}," +
                                $"{record.panelPosX:F3},{record.panelPosY:F3},{record.panelPosZ:F3}," +
                                $"{record.incidentAngle:F2},{record.reflectionAngle:F2},{record.reflectionError:F2}," +
                                $"{record.steeringAngle:F2},{record.specularStatus}," +
                                $"{record.distanceTxToIRS:F3},{record.distanceIRSToRX:F3},{record.totalDistance:F3}"
                            );
                        }

                        UpdateBestPath(record);
                    }
                }
            }

            writerAll?.Flush();
            writerAll?.Close();
            writerAll = null;

            // ---------- 2) 최소 steering 경로 CSV + 통계 ----------
            int totalMinLinks = 0;
            int countLe15 = 0;
            int countLe20 = 0;
            int countLe25 = 0;
            double sumDistance = 0.0;

            using (var writerMin = new StreamWriter(minFilePath, false, Encoding.UTF8))
            {
                writerMin.WriteLine(MinPathsHeader);

                foreach (var kvTx in _bestPathByTxRx)
                {
                    int txId = kvTx.Key;
                    var rxDict = kvTx.Value;

                    foreach (var kvRx in rxDict)
                    {
                        BestPathRecord rec = kvRx.Value;

                        writerMin.WriteLine(
                            $"{rec.txRackId},{rec.rxRackId},{rec.irsId}," +
                            $"{rec.txPosX:F3},{rec.txPosY:F3},{rec.txPosZ:F3}," +
                            $"{rec.rxPosX:F3},{rec.rxPosY:F3},{rec.rxPosZ:F3}," +
                            $"{rec.panelPosX:F3},{rec.panelPosY:F3},{rec.panelPosZ:F3}," +
                            $"{rec.incidentAngle:F2},{rec.reflectionAngle:F2},{rec.reflectionError:F2}," +
                            $"{rec.steeringAngle:F2},{rec.specularStatus}," +
                            $"{rec.distanceTxToIRS:F3},{rec.distanceIRSToRX:F3},{rec.totalDistance:F3}"
                        );

                        totalMinLinks++;
                        float angle = rec.steeringAngle;

                        if (angle <= 15f) countLe15++;
                        if (angle <= 20f) countLe20++;
                        if (angle <= 25f) countLe25++;

                        sumDistance += rec.totalDistance;
                    }
                }
            }

            float ratioLe15 = 0f, ratioLe20 = 0f, ratioLe25 = 0f;
            float avgDistance = 0f;

            using (var writerStats = new StreamWriter(statsFilePath, false, Encoding.UTF8))
            {
                writerStats.WriteLine("ThresholdDeg,Count,RatioPercent");

                if (totalMinLinks > 0)
                {
                    ratioLe15 = (float)countLe15 / totalMinLinks * 100f;
                    ratioLe20 = (float)countLe20 / totalMinLinks * 100f;
                    ratioLe25 = (float)countLe25 / totalMinLinks * 100f;
                    avgDistance = (float)(sumDistance / totalMinLinks);

                    writerStats.WriteLine($"15,{countLe15},{ratioLe15:F2}");
                    writerStats.WriteLine($"20,{countLe20},{ratioLe20:F2}");
                    writerStats.WriteLine($"25,{countLe25},{ratioLe25:F2}");

                    if (verboseLog)
                    {
                        Debug.Log($"[IrsHeightSweep][{labelForLogs}] Total links (best per Tx-Rx): {totalMinLinks}");
                        Debug.Log($"[IrsHeightSweep][{labelForLogs}]   <=15° : {countLe15} ({ratioLe15:F2}%)");
                        Debug.Log($"[IrsHeightSweep][{labelForLogs}]   <=20° : {countLe20} ({ratioLe20:F2}%)");
                        Debug.Log($"[IrsHeightSweep][{labelForLogs}]   <=25° : {countLe25} ({ratioLe25:F2}%)");
                        Debug.Log($"[IrsHeightSweep][{labelForLogs}]   Avg distance (m): {avgDistance:F3}");
                    }
                }
                else
                {
                    writerStats.WriteLine("15,0,0");
                    writerStats.WriteLine("20,0,0");
                    writerStats.WriteLine("25,0,0");
                }
            }

            return new HeightSummary
            {
                height = boardHeight,
                threshold15 = ratioLe15,
                threshold20 = ratioLe20,
                threshold25 = ratioLe25,
                avgDistance = avgDistance
            };
        }
        finally
        {
            if (writerAll != null)
            {
                writerAll.Flush();
                writerAll.Close();
            }
        }
    }

    /// <summary>
    /// 단일 (Tx, Rx, Panel) 경로에 대해 각도/거리 값을 계산한다.
    /// </summary>
    private BestPathRecord EvaluateSinglePath(
    Transform tx, Transform rx, Transform panel,
    int txId, int rxId, int panelId)
    {
        Vector3 txPos = tx.position;
        Vector3 rxPos = rx.position;
        Vector3 panelPos = panel.position;

        // Tx -> Panel, Panel -> Rx
        Vector3 incomingDir = (panelPos - txPos).normalized;
        Vector3 outgoingDir = (rxPos - panelPos).normalized;

        Vector3 panelNormal = panel.TransformDirection(panelLocalNormal).normalized;

        // 법선 기준 입사/반사 각도 (기존 그대로 사용해도 OK)
        float incidentAngle = Vector3.Angle(-incomingDir, panelNormal);
        float reflectionAngle = Vector3.Angle(outgoingDir, panelNormal);

        // 여기가 핵심 수정: -incomingDir 대신 incomingDir 사용
        Vector3 idealOutgoing = Vector3.Reflect(incomingDir, panelNormal).normalized;
        float steeringAngle = Vector3.Angle(idealOutgoing, outgoingDir);

        float reflectionError = Mathf.Abs(incidentAngle - reflectionAngle);

        string status = (steeringAngle <= reflectionAngleToleranceDeg)
            ? "Specular"
            : "Steering";

        float dTxToPanel = Vector3.Distance(txPos, panelPos);
        float dPanelToRx = Vector3.Distance(panelPos, rxPos);
        float totalDist = dTxToPanel + dPanelToRx;

        BestPathRecord record = new BestPathRecord
        {
            txRackId = txId,
            rxRackId = rxId,
            irsId = panelId,

            // positions
            txPosX = txPos.x,
            txPosY = txPos.y,
            txPosZ = txPos.z,

            rxPosX = rxPos.x,
            rxPosY = rxPos.y,
            rxPosZ = rxPos.z,

            panelPosX = panelPos.x,
            panelPosY = panelPos.y,
            panelPosZ = panelPos.z,

            // angles
            incidentAngle = incidentAngle,
            reflectionAngle = reflectionAngle,
            reflectionError = reflectionError,
            steeringAngle = steeringAngle,
            specularStatus = status,

            // distances
            distanceTxToIRS = dTxToPanel,
            distanceIRSToRX = dPanelToRx,
            totalDistance = totalDist
        };

        return record;
    }


    /// <summary>
    /// (Tx,Rx) 쌍에 대해 steeringAngle이 더 작은 경로만 남기도록 갱신.
    /// </summary>
    private void UpdateBestPath(BestPathRecord record)
    {
        if (!_bestPathByTxRx.TryGetValue(record.txRackId, out var rxDict))
        {
            rxDict = new Dictionary<int, BestPathRecord>();
            _bestPathByTxRx[record.txRackId] = rxDict;
        }

        if (rxDict.TryGetValue(record.rxRackId, out var existing))
        {
            if (record.steeringAngle < existing.steeringAngle)
                rxDict[record.rxRackId] = record;
        }
        else
        {
            rxDict[record.rxRackId] = record;
        }
    }

    /// <summary>
    /// 씬에서 트랜시버/패널 리스트를 수집한다.
    /// </summary>
    private void CollectTransceiversAndPanels()
    {
        _transceivers.Clear();
        _panels.Clear();

        if (racksRoot != null)
        {
            foreach (Transform child in racksRoot)
            {
                if (!child.gameObject.activeInHierarchy)
                    continue;

                Transform tr = string.IsNullOrEmpty(transceiverChildName)
                    ? child            // child 자체를 트랜시버로 사용
                    : child.Find(transceiverChildName);

                if (tr != null)
                    _transceivers.Add(tr);
            }
        }

        if (panelsRoot != null)
        {
            foreach (Transform child in panelsRoot)
            {
                if (!child.gameObject.activeInHierarchy)
                    continue;

                // 이름 prefix로 패널만 선택 (IRS_Backplane 등은 자동 스킵)
                if (!string.IsNullOrEmpty(panelNamePrefix) &&
                    !child.name.StartsWith(panelNamePrefix, StringComparison.OrdinalIgnoreCase))
                    continue;

                _panels.Add(child);
            }
        }

        if (verboseLog)
        {
            Debug.Log($"[IrsHeightSweep] Collected {_transceivers.Count} transceivers and {_panels.Count} panels.");
        }
    }


    /// <summary>
    /// 높이별 요약 결과 CSV를 작성한다.
    /// </summary>
    private void WriteHeightSummaryCsv(string filePath, IList<HeightSummary> summaries)
    {
        using (var writer = new StreamWriter(filePath, false, Encoding.UTF8))
        {
            writer.WriteLine("Height,Threshold_Deg 15,Threshold_Deg 20,Threshold_Deg 25,Distance");

            if (summaries != null)
            {
                foreach (var s in summaries)
                {
                    writer.WriteLine(
                        $"{s.height:F2},{s.threshold15:F2},{s.threshold20:F2},{s.threshold25:F2},{s.avgDistance:F8}"
                    );
                }
            }
        }
    }

    /// <summary>
    /// (Tx,Rx,Panel) 경로 하나에 대한 요약 레코드.
    /// Gaussian 파라미터를 나중에 추가하고 싶으면 여기에 필드를 늘리면 된다.
    /// </summary>
    [Serializable]
    public struct BestPathRecord
    {
        public int txRackId;
        public int rxRackId;
        public int irsId;

        // World positions
        public float txPosX;
        public float txPosY;
        public float txPosZ;

        public float rxPosX;
        public float rxPosY;
        public float rxPosZ;

        public float panelPosX;
        public float panelPosY;
        public float panelPosZ;

        // Angles
        public float incidentAngle;
        public float reflectionAngle;
        public float reflectionError;
        public float steeringAngle;
        public string specularStatus;

        // Distances
        public float distanceTxToIRS;
        public float distanceIRSToRX;
        public float totalDistance;
    }


    /// <summary>
    /// 높이별 최종 요약: 임계각 비율과 평균 거리.
    /// </summary>
    [Serializable]
    public struct HeightSummary
    {
        public float height;
        public float threshold15;
        public float threshold20;
        public float threshold25;
        public float avgDistance;
    }

#if UNITY_EDITOR
[UnityEditor.CustomEditor(typeof(IrsHeightSweepExperiment))]
public class IrsHeightSweepExperimentEditor : UnityEditor.Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        IrsHeightSweepExperiment exp = (IrsHeightSweepExperiment)target;

        GUILayout.Space(10);

        if (GUILayout.Button("Run IRS Height Sweep Experiment"))
        {
            exp.RunExperiment();
        }
    }
}
#endif

}
