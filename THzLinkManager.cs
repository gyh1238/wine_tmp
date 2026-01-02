using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// THz / r-WINE interference experiment (standalone; no FSOLinkManager needed).
/// - Randomly creates many r-WINE links (Tx -> IRS element -> Rx)
/// - Instantiates Beam Prefab and calls GaussianBeam.Initialize() to build the cone-like MeshCollider
/// - Detects interference by checking the ACTUAL beam MeshCollider against OTHER transceiver colliders
///   using Physics.ComputePenetration (no cylinder/sampling approximation).
/// </summary>
public class THzLinkManager : MonoBehaviour
{
    // -----------------------------
    // Link Definition
    // -----------------------------
    [Serializable]
    public class LinkDefinition
    {
        public string name;
        public int transmitterIndex;
        public int receiverIndex;
        public int reflectingElementIndex; // r-WINE only (>=0)
    }

    // -----------------------------
    // Dependencies (same concept as FSOLinkManager but owned here)
    // -----------------------------
    [Header("Link Setup")]
    [Tooltip("Prefab that has GaussianBeam + GaussianBeamCollider + LineRenderer components.")]
    public GameObject beamPrefab;

    [Header("Dependencies")]
    public ProceduralTransceiverPlacer transceiverPlacer;
    public ProceduralIRSPlacer irsPlacer;

    [Header("Gaussian Beam Parameters")]
    [Tooltip("w0 (meters)")]
    public float beamWaist = 0.002f;

    [Tooltip("lambda (meters)")]
    public float wavelength = 1550e-9f;

    [Header("Links (auto-filled by experiment)")]
    public List<LinkDefinition> links = new List<LinkDefinition>();

    // -----------------------------
    // Experiment settings
    // -----------------------------
    [Header("Random r-WINE Generation")]
    [Min(1)] public int linkCount = 200;
    public int seed = 123;

    [Tooltip("Avoid duplicate segment names (Tx,Refl) and (Refl,Rx). Recommended.")]
    public bool enforceUniqueSegments = true;

    [Header("Interference Detection (Collider-based)")]
    [Tooltip("If true, only check colliders under Lens_Aperture_Mesh. If false, all colliders under each _Transceiver root are checked.")]
    public bool checkOnlyLensAperture = true;

    [Tooltip("Name of lens object used for aiming and (optionally) collider filtering.")]
    public string lensObjectName = "Lens_Aperture_Mesh";

    [Tooltip("Only consider colliders on these layers as potential interferers (transceiver side).")]
    public LayerMask transceiverColliderMask = ~0;

    [Tooltip("Early exit on first interferer per segment (faster). If false, collects all interferers for that segment.")]
    public bool earlyExitPerSegment = true;

    [Header("Beam Runtime Behavior")]
    [Tooltip("Disable GaussianBeam script after Initialize to suppress OnTriggerEnter blockage logs (MachineRoom/IRS etc). MeshCollider remains for penetration tests.")]
    public bool disableGaussianBeamScriptAfterInit = true;

    [Header("Logging")]
    public bool logInterferedLinks = true;
    public int logTopKInterferers = 10;

    // -----------------------------
    // Internal runtime caches
    // -----------------------------
    private readonly List<GaussianBeam> _activeBeams = new List<GaussianBeam>(4096);

    // Cache of candidate colliders on transceivers
    private readonly List<Collider> _candidateTransceiverColliders = new List<Collider>(8192);
    private readonly Dictionary<Collider, Transform> _colliderToTransceiverRoot = new Dictionary<Collider, Transform>(8192);

    // -----------------------------
    // Public entry point
    // -----------------------------
    [ContextMenu("Run r-WINE Interference Experiment")]
    public void Run()
    {
        if (!Application.isPlaying)
        {
            Debug.LogError("Run this in Play Mode.", this);
            return;
        }

        StartCoroutine(RunCoroutine());
    }

    private IEnumerator RunCoroutine()
    {
        // 0) Validate
        if (beamPrefab == null)
        {
            Debug.LogError("Beam Prefab is not assigned.", this);
            yield break;
        }
        if (transceiverPlacer == null)
        {
            Debug.LogError("Transceiver Placer is not assigned.", this);
            yield break;
        }
        if (irsPlacer == null)
        {
            Debug.LogError("IRS Placer is not assigned.", this);
            yield break;
        }

        Transform transContainer = transceiverPlacer.transform;
        int txrxCount = transContainer.childCount;

        int irsCount = irsPlacer.elementCountX * irsPlacer.elementCountZ;
        if (txrxCount <= 1 || irsCount <= 0)
        {
            Debug.LogError($"Invalid counts. Transceivers={txrxCount}, IRS elements={irsCount}", this);
            yield break;
        }

        // 1) Build random links (r-WINE only)
        BuildRandomLinks(txrxCount, irsCount);

        // 2) Generate beams (standalone, like FSOLinkManager.GenerateLinks but inside THzLinkManager)
        GenerateLinks();

        // Wait a couple physics ticks so colliders are registered
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // 3) Cache transceiver roots + set
        var transceiverRoots = new List<Transform>(txrxCount);
        for (int i = 0; i < txrxCount; i++)
            transceiverRoots.Add(transContainer.GetChild(i));
        var transceiverRootSet = new HashSet<Transform>(transceiverRoots);

        // 4) Build candidate collider list (transceiver-only; IRS/environment ignored by design)
        BuildCandidateTransceiverColliders(transceiverRoots, transceiverRootSet);

        // 5) Map beams by name for lookup
        var beamByName = new Dictionary<string, GaussianBeam>(StringComparer.Ordinal);
        foreach (var b in _activeBeams)
        {
            if (b == null) continue;
            if (!beamByName.ContainsKey(b.name))
                beamByName.Add(b.name, b);
        }

        // 6) For each link, check two segments by REAL collider overlap
        int total = 0;
        int interfered = 0;

        var interfererFreq = new Dictionary<string, int>(StringComparer.Ordinal);

        foreach (var link in links)
        {
            if (link == null) continue;
            if (link.reflectingElementIndex < 0) continue;

            total++;

            string seg1Name = $"rWINE_Tx{link.transmitterIndex}_Refl{link.reflectingElementIndex}";
            string seg2Name = $"rWINE_Refl{link.reflectingElementIndex}_Rx{link.receiverIndex}";

            if (!beamByName.TryGetValue(seg1Name, out var seg1) || !beamByName.TryGetValue(seg2Name, out var seg2))
                continue;

            Transform intendedTx = SafeGetChild(transContainer, link.transmitterIndex);
            Transform intendedRx = SafeGetChild(transContainer, link.receiverIndex);

            bool seg1Interf = CheckBeamInterferenceByCollider(seg1, intendedTx, intendedRx, out var i1);
            bool seg2Interf = CheckBeamInterferenceByCollider(seg2, intendedTx, intendedRx, out var i2);

            bool linkInterf = seg1Interf || seg2Interf;
            if (!linkInterf) continue;

            interfered++;

            if (logInterferedLinks)
                Debug.Log($"[Interference] {link.name} (seg1={seg1Interf}, seg2={seg2Interf})");

            foreach (var t in i1.Concat(i2))
            {
                if (t == null) continue;
                interfererFreq.TryGetValue(t.name, out int c);
                interfererFreq[t.name] = c + 1;
            }
        }

        float ratio = total > 0 ? (float)interfered / total : 0f;
        Debug.Log($"[r-WINE Interference Summary] total_links={total}, interfered_links={interfered}, ratio={ratio:P2}");

        foreach (var kv in interfererFreq.OrderByDescending(x => x.Value).Take(Mathf.Max(0, logTopKInterferers)))
            Debug.Log($"[Top Interferer] {kv.Key} : {kv.Value} hits");
    }

    // -----------------------------
    // Link + Beam generation
    // -----------------------------
    private void BuildRandomLinks(int txrxCount, int irsCount)
    {
        links.Clear();

        var rng = new System.Random(seed);

        var usedTxRefl = new HashSet<(int, int)>();
        var usedReflRx = new HashSet<(int, int)>();
        var usedTriple = new HashSet<(int, int, int)>();

        int guard = 0;
        while (links.Count < linkCount && guard < linkCount * 50)
        {
            guard++;

            int tx = rng.Next(0, txrxCount);
            int rx = rng.Next(0, txrxCount);
            if (rx == tx) continue;

            int refl = rng.Next(0, irsCount);

            if (enforceUniqueSegments)
            {
                if (usedTxRefl.Contains((tx, refl))) continue;
                if (usedReflRx.Contains((refl, rx))) continue;
            }

            if (usedTriple.Contains((tx, refl, rx))) continue;

            usedTxRefl.Add((tx, refl));
            usedReflRx.Add((refl, rx));
            usedTriple.Add((tx, refl, rx));

            links.Add(new LinkDefinition
            {
                transmitterIndex = tx,
                receiverIndex = rx,
                reflectingElementIndex = refl,
                name = $"rWINE_Tx{tx}_Refl{refl}_Rx{rx}"
            });
        }

        if (links.Count < linkCount)
        {
            Debug.LogWarning($"Requested {linkCount} links, generated {links.Count}. " +
                             $"(Try disabling Enforce Unique Segments or increasing IRS/Transceiver counts)", this);
        }
    }

    /// <summary>
    /// Clears old beams and builds new ones for current 'links' list.
    /// Creates 2 segments per r-WINE: Tx->Refl and Refl->Rx.
    /// </summary>
    public void GenerateLinks()
    {
        // Destroy previously created beam GameObjects (children under this manager)
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            var ch = transform.GetChild(i);
            if (ch == null) continue;
            Destroy(ch.gameObject);
        }
        _activeBeams.Clear();

        Transform transContainer = transceiverPlacer.transform;

        foreach (var link in links)
        {
            if (link == null) continue;

            // Validate indices
            if (link.transmitterIndex < 0 || link.receiverIndex < 0) continue;
            if (link.transmitterIndex >= transContainer.childCount || link.receiverIndex >= transContainer.childCount) continue;
            if (link.reflectingElementIndex < 0) continue;

            Transform transmitter = transContainer.GetChild(link.transmitterIndex);
            Transform receiver = transContainer.GetChild(link.receiverIndex);
            Transform reflectingElement = irsPlacer.GetIRSElement(link.reflectingElementIndex);
            if (transmitter == null || receiver == null || reflectingElement == null) continue;

            // Lens transforms (for accurate start/end after aiming)
            Transform txLens = FindDeepChild(transmitter, lensObjectName);
            Transform rxLens = FindDeepChild(receiver, lensObjectName);
            if (txLens == null || rxLens == null)
            {
                Debug.LogError($"Could not find '{lensObjectName}' in {link.name}. Check your transceiver prefab hierarchy.", this);
                continue;
            }

            // Aim Tx and Rx toward reflecting element
            AimTransceiver(transmitter, reflectingElement.position);
            AimTransceiver(receiver, reflectingElement.position);

            Vector3 startPos = txLens.position;
            Vector3 endPos = rxLens.position;

            // Segment 1: Tx -> Refl
            GameObject beamObj1 = Instantiate(beamPrefab, transform);
            beamObj1.name = $"rWINE_Tx{link.transmitterIndex}_Refl{link.reflectingElementIndex}";
            GaussianBeam beam1 = beamObj1.GetComponent<GaussianBeam>();
            if (beam1 == null)
            {
                Debug.LogError($"Beam Prefab missing GaussianBeam component. ({beamObj1.name})", this);
                Destroy(beamObj1);
                continue;
            }
            beam1.Initialize(startPos, reflectingElement.position, transmitter, reflectingElement, beamWaist, wavelength);
            _activeBeams.Add(beam1);

            // Segment 2: Refl -> Rx
            GameObject beamObj2 = Instantiate(beamPrefab, transform);
            beamObj2.name = $"rWINE_Refl{link.reflectingElementIndex}_Rx{link.receiverIndex}";
            GaussianBeam beam2 = beamObj2.GetComponent<GaussianBeam>();
            if (beam2 == null)
            {
                Debug.LogError($"Beam Prefab missing GaussianBeam component. ({beamObj2.name})", this);
                Destroy(beamObj2);
                continue;
            }
            beam2.Initialize(reflectingElement.position, endPos, reflectingElement, receiver, beamWaist, wavelength);
            _activeBeams.Add(beam2);

            // Optional visuals: r-WINE as red
            var lr1 = beamObj1.GetComponent<LineRenderer>();
            var lr2 = beamObj2.GetComponent<LineRenderer>();
            if (lr1 != null) lr1.material.color = Color.red;
            if (lr2 != null) lr2.material.color = Color.red;

            // Disable GaussianBeam script to suppress OnTriggerEnter blockage logs,
            // but keep its MeshCollider for penetration-based interference tests.
            if (disableGaussianBeamScriptAfterInit)
            {
                beam1.enabled = false;
                beam2.enabled = false;
            }
        }
    }

    // -----------------------------
    // Interference detection (REAL collider overlap)
    // -----------------------------
    private bool CheckBeamInterferenceByCollider(
        GaussianBeam beam,
        Transform intendedTx,
        Transform intendedRx,
        out List<Transform> interferers)
    {
        interferers = new List<Transform>();
        if (beam == null) return false;

        // Beam collider is generated by GaussianBeam.Initialize via GaussianBeamCollider.UpdateBeamParameters
        var beamMeshCol = beam.GetComponent<MeshCollider>();
        if (beamMeshCol == null || !beamMeshCol.enabled) return false;

        Bounds beamBounds = beamMeshCol.bounds;

        for (int i = 0; i < _candidateTransceiverColliders.Count; i++)
        {
            Collider targetCol = _candidateTransceiverColliders[i];
            if (targetCol == null || !targetCol.enabled) continue;

            // LayerMask filter for candidates
            if (!IsInLayerMask(targetCol.gameObject.layer, transceiverColliderMask)) continue;

            // Quick bounds prefilter
            if (!beamBounds.Intersects(targetCol.bounds)) continue;

            // Map collider -> transceiver root
            if (!_colliderToTransceiverRoot.TryGetValue(targetCol, out Transform hitRoot) || hitRoot == null) continue;

            // Exclude intended endpoints
            if (hitRoot == intendedTx || hitRoot == intendedRx) continue;

            // Actual collider overlap test (uses REAL shapes)
            bool overlapped = Physics.ComputePenetration(
                beamMeshCol, beamMeshCol.transform.position, beamMeshCol.transform.rotation,
                targetCol, targetCol.transform.position, targetCol.transform.rotation,
                out _, out _);

            if (!overlapped) continue;

            if (!interferers.Contains(hitRoot))
                interferers.Add(hitRoot);

            if (earlyExitPerSegment)
                return true;
        }

        return interferers.Count > 0;
    }

    private void BuildCandidateTransceiverColliders(List<Transform> transceiverRoots, HashSet<Transform> transceiverRootSet)
    {
        _candidateTransceiverColliders.Clear();
        _colliderToTransceiverRoot.Clear();

        foreach (var root in transceiverRoots)
        {
            if (root == null) continue;
            if (!transceiverRootSet.Contains(root)) continue;

            if (checkOnlyLensAperture)
            {
                Transform lens = FindDeepChild(root, lensObjectName);
                if (lens == null) continue;

                foreach (var col in lens.GetComponentsInChildren<Collider>(true))
                {
                    if (col == null) continue;
                    _candidateTransceiverColliders.Add(col);
                    _colliderToTransceiverRoot[col] = root;
                }
            }
            else
            {
                foreach (var col in root.GetComponentsInChildren<Collider>(true))
                {
                    if (col == null) continue;
                    _candidateTransceiverColliders.Add(col);
                    _colliderToTransceiverRoot[col] = root;
                }
            }
        }
    }

    // -----------------------------
    // Aiming (copied conceptually from FSOLinkManager)
    // -----------------------------
    private void AimTransceiver(Transform transceiverRoot, Vector3 targetPosition)
    {
        Transform basePivot = transceiverRoot.Find("Gimbal_Base_Pivot");
        Transform yokePivot = basePivot != null ? basePivot.Find("Gimbal_Yoke_Pivot") : null;

        if (basePivot == null || yokePivot == null) return;

        // Yaw (around Y) at base pivot
        Vector3 targetDir = targetPosition - basePivot.position;
        targetDir.y = 0f; // yaw only
        if (targetDir.sqrMagnitude > 1e-8f)
        {
            Quaternion yawRot = Quaternion.LookRotation(targetDir.normalized, Vector3.up);
            basePivot.rotation = yawRot;
        }

        // Pitch (around local X) at yoke pivot
        Vector3 localTargetDir = targetPosition - yokePivot.position;
        Vector3 localDirInYoke = yokePivot.InverseTransformDirection(localTargetDir.normalized);
        float pitch = Mathf.Atan2(localDirInYoke.y, localDirInYoke.z) * Mathf.Rad2Deg;
        yokePivot.localRotation = Quaternion.Euler(-pitch, 0f, 0f);
    }

    // -----------------------------
    // Helpers
    // -----------------------------
    private static Transform SafeGetChild(Transform parent, int index)
    {
        if (parent == null) return null;
        if (index < 0 || index >= parent.childCount) return null;
        return parent.GetChild(index);
    }

    // Finds a descendant by name (no dependency on any extension method)
    private static Transform FindDeepChild(Transform parent, string childName)
    {
        if (parent == null) return null;

        for (int i = 0; i < parent.childCount; i++)
        {
            Transform ch = parent.GetChild(i);
            if (ch.name == childName) return ch;

            Transform found = FindDeepChild(ch, childName);
            if (found != null) return found;
        }
        return null;
    }

    private static bool IsInLayerMask(int layer, LayerMask mask)
    {
        return (mask.value & (1 << layer)) != 0;
    }
}
