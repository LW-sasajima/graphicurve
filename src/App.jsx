import { useState, useRef, useEffect, useCallback, useMemo } from "react";
import * as opentype from "opentype.js";

/* ═══════════════════════════════════════════
   MATH: Bézier evaluation & curvature
   ═══════════════════════════════════════════ */

function cubicPt(p0, p1, p2, p3, t) {
  const s = 1 - t;
  return {
    x: s*s*s*p0.x + 3*s*s*t*p1.x + 3*s*t*t*p2.x + t*t*t*p3.x,
    y: s*s*s*p0.y + 3*s*s*t*p1.y + 3*s*t*t*p2.y + t*t*t*p3.y,
  };
}
function cubicD1(p0, p1, p2, p3, t) {
  const s = 1 - t;
  return {
    x: 3*(s*s*(p1.x-p0.x) + 2*s*t*(p2.x-p1.x) + t*t*(p3.x-p2.x)),
    y: 3*(s*s*(p1.y-p0.y) + 2*s*t*(p2.y-p1.y) + t*t*(p3.y-p2.y)),
  };
}
function cubicD2(p0, p1, p2, p3, t) {
  const s = 1 - t;
  return {
    x: 6*(s*(p2.x-2*p1.x+p0.x) + t*(p3.x-2*p2.x+p1.x)),
    y: 6*(s*(p2.y-2*p1.y+p0.y) + t*(p3.y-2*p2.y+p1.y)),
  };
}
function quadPt(p0, p1, p2, t) {
  const s = 1 - t;
  return { x: s*s*p0.x + 2*s*t*p1.x + t*t*p2.x, y: s*s*p0.y + 2*s*t*p1.y + t*t*p2.y };
}
function quadD1(p0, p1, p2, t) {
  const s = 1 - t;
  return { x: 2*(s*(p1.x-p0.x) + t*(p2.x-p1.x)), y: 2*(s*(p1.y-p0.y) + t*(p2.y-p1.y)) };
}
function quadD2(p0, p1, p2) {
  return { x: 2*(p2.x-2*p1.x+p0.x), y: 2*(p2.y-2*p1.y+p0.y) };
}

function curvature(d1, d2) {
  const cross = d1.x * d2.y - d1.y * d2.x;
  const speed = Math.sqrt(d1.x*d1.x + d1.y*d1.y);
  if (speed < 1e-10) return 0;
  return cross / (speed * speed * speed);
}

function evalSegment(seg, t) {
  if (seg.type === "cubic") {
    const pt = cubicPt(seg.p0, seg.p1, seg.p2, seg.p3, t);
    const d1 = cubicD1(seg.p0, seg.p1, seg.p2, seg.p3, t);
    const d2 = cubicD2(seg.p0, seg.p1, seg.p2, seg.p3, t);
    const k = curvature(d1, d2);
    return { ...pt, d1, d2, k };
  }
  if (seg.type === "quad") {
    const pt = quadPt(seg.p0, seg.p1, seg.p2, t);
    const d1 = quadD1(seg.p0, seg.p1, seg.p2, t);
    const d2 = quadD2(seg.p0, seg.p1, seg.p2);
    const k = curvature(d1, d2);
    return { ...pt, d1, d2, k };
  }
  // line
  const pt = { x: seg.p0.x + t*(seg.p1.x - seg.p0.x), y: seg.p0.y + t*(seg.p1.y - seg.p0.y) };
  const d1 = { x: seg.p1.x - seg.p0.x, y: seg.p1.y - seg.p0.y };
  return { ...pt, d1, d2: {x:0,y:0}, k: 0 };
}

function osculatingCircle(pt, d1, k) {
  if (Math.abs(k) < 1e-8) return null;
  const R = 1 / Math.abs(k);
  const speed = Math.sqrt(d1.x*d1.x + d1.y*d1.y);
  if (speed < 1e-10) return null;
  const nx = -d1.y / speed;
  const ny = d1.x / speed;
  const sign = k > 0 ? 1 : -1;
  return { cx: pt.x + sign * R * nx, cy: pt.y + sign * R * ny, r: R };
}

// Find inflection points for cubic Bézier (κ=0 → cross product = 0)
function cubicInflections(p0, p1, p2, p3) {
  const results = [];
  const N = 200;
  let prevCross = null;
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    const d1 = cubicD1(p0, p1, p2, p3, t);
    const d2 = cubicD2(p0, p1, p2, p3, t);
    const cross = d1.x * d2.y - d1.y * d2.x;
    if (prevCross !== null && prevCross * cross < 0) {
      // Bisect
      let lo = (i-1)/N, hi = t;
      for (let j = 0; j < 20; j++) {
        const mid = (lo + hi) / 2;
        const md1 = cubicD1(p0, p1, p2, p3, mid);
        const md2 = cubicD2(p0, p1, p2, p3, mid);
        const mc = md1.x * md2.y - md1.y * md2.x;
        if (mc * prevCross < 0) hi = mid; else lo = mid;
      }
      const tInf = (lo + hi) / 2;
      if (tInf > 0.001 && tInf < 0.999) results.push(tInf);
    }
    prevCross = cross;
  }
  return results;
}

// Find curvature extrema (dκ/dt = 0) numerically
function findCurvatureExtrema(seg, N = 200) {
  const results = [];
  let prevDk = null;
  let prevK = null;
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    const e = evalSegment(seg, t);
    if (prevK !== null) {
      const dk = e.k - prevK;
      if (prevDk !== null && prevDk * dk < 0 && Math.abs(e.k) > 1e-6) {
        results.push({ t: (i - 0.5) / N, k: (e.k + prevK) / 2 });
      }
      prevDk = dk;
    }
    prevK = e.k;
  }
  return results;
}

/* ═══════════════════════════════════════════
   PATH PARSING
   ═══════════════════════════════════════════ */

function parseGlyphPath(commands) {
  const segments = [];
  let cx = 0, cy = 0, sx = 0, sy = 0;
  for (const cmd of commands) {
    if (cmd.type === "M") {
      cx = cmd.x; cy = cmd.y; sx = cmd.x; sy = cmd.y;
    } else if (cmd.type === "L") {
      segments.push({ type: "line", p0: {x:cx,y:cy}, p1: {x:cmd.x,y:cmd.y} });
      cx = cmd.x; cy = cmd.y;
    } else if (cmd.type === "Q") {
      segments.push({ type: "quad", p0: {x:cx,y:cy}, p1: {x:cmd.x1,y:cmd.y1}, p2: {x:cmd.x,y:cmd.y} });
      cx = cmd.x; cy = cmd.y;
    } else if (cmd.type === "C") {
      segments.push({ type: "cubic", p0: {x:cx,y:cy}, p1: {x:cmd.x1,y:cmd.y1}, p2: {x:cmd.x2,y:cmd.y2}, p3: {x:cmd.x,y:cmd.y} });
      cx = cmd.x; cy = cmd.y;
    } else if (cmd.type === "Z") {
      if (Math.abs(cx - sx) > 0.1 || Math.abs(cy - sy) > 0.1) {
        segments.push({ type: "line", p0: {x:cx,y:cy}, p1: {x:sx,y:sy} });
      }
      cx = sx; cy = sy;
    }
  }
  return segments;
}

function analyzeSegments(segments, samplesPerSeg = 60) {
  return segments.map((seg, idx) => {
    const samples = [];
    for (let i = 0; i <= samplesPerSeg; i++) {
      const t = i / samplesPerSeg;
      samples.push({ t, ...evalSegment(seg, t) });
    }
    let inflections = [];
    if (seg.type === "cubic") inflections = cubicInflections(seg.p0, seg.p1, seg.p2, seg.p3);

    const extrema = (seg.type !== "line") ? findCurvatureExtrema(seg, samplesPerSeg * 2) : [];

    return { ...seg, idx, samples, inflections, extrema };
  });
}

/* ═══════════════════════════════════════════
   CURVATURE COLORMAP
   ═══════════════════════════════════════════ */

function kColor(k, maxK) {
  if (Math.abs(k) < 1e-8) return "#666";
  const norm = Math.min(1, Math.abs(k) / maxK);
  if (k > 0) {
    const r = Math.round(40 + 200 * norm);
    const g = Math.round(120 - 60 * norm);
    const b = Math.round(220 - 100 * norm);
    return `rgb(${r},${g},${b})`;
  } else {
    const r = Math.round(220 - 80 * norm);
    const g = Math.round(140 + 60 * norm);
    const b = Math.round(40 + 60 * norm);
    return `rgb(${r},${g},${b})`;
  }
}

/* ═══════════════════════════════════════════
   MAIN COMPONENT
   ═══════════════════════════════════════════ */

/* ═══════════════════════════════════════════
   DESIGN TOKENS (Apple HIG-inspired)
   ═══════════════════════════════════════════ */
const T = {
  // Backgrounds (macOS dark)
  bgWindow: "#1e1e1e",
  bgContent: "#252525",
  bgSidebar: "rgba(40,40,40,0.72)",
  bgToolbar: "rgba(36,36,36,0.85)",
  bgGroup: "#2c2c2e",
  bgGroupHover: "#3a3a3c",
  bgField: "#1c1c1e",

  // Separators / borders
  separator: "rgba(255,255,255,0.08)",
  border: "rgba(255,255,255,0.12)",

  // Text (DADS-tuned: brighter tertiary for kanji legibility on dark bg)
  labelPrimary: "rgba(255,255,255,0.96)",
  labelSecondary: "rgba(235,235,245,0.72)",
  labelTertiary: "rgba(235,235,245,0.50)",
  labelQuaternary: "rgba(235,235,245,0.28)",

  // Accent (systemBlue)
  accent: "#0A84FF",
  accentHover: "#409CFF",

  // Semantic colors
  red: "#FF453A",
  yellow: "#FFD60A",
  green: "#30D158",
  teal: "#64D2FF",

  // Geometry
  radiusCard: 10,
  radiusControl: 6,
  radiusInline: 4,

  // Typography (DADS: latin first, then Japanese, with Hiragino/BIZ UDP/Yu/Meiryo fallbacks)
  fontUI: '-apple-system, BlinkMacSystemFont, "SF Pro Text", "SF Pro Display", "Helvetica Neue", "Hiragino Sans", "Hiragino Kaku Gothic ProN", "BIZ UDPGothic", "Yu Gothic UI", "Yu Gothic", Meiryo, sans-serif',
  fontMono: '"SF Mono", ui-monospace, Menlo, Monaco, Consolas, "BIZ UDGothic", "Hiragino Sans", monospace',

  // Type scale (DADS: min 12px, body 14px, line-height 1.5+ for JP)
  fzCaption: 12,    // 補足情報
  fzFootnote: 13,   // 副次的なラベル
  fzBody: 14,       // 標準テキスト
  fzCallout: 15,    // 強調テキスト
  fzHeadline: 16,   // 見出し
  lhTight: 1.45,
  lhBody: 1.6,
  // Letter-spacing for mixed JP/Latin
  trackJP: "0.02em",
};

function Row({ label, value, help }) {
  return (
    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", padding: "5px 0", fontSize: T.fzFootnote, lineHeight: T.lhTight }}>
      <span style={{ color: T.labelSecondary, fontFamily: T.fontUI, display: "inline-flex", alignItems: "center", letterSpacing: T.trackJP }}>
        {label}
        {help && <HelpButton help={help} />}
      </span>
      <span style={{ color: T.labelPrimary, fontVariantNumeric: "tabular-nums", fontFamily: T.fontMono, fontSize: T.fzCaption }}>{value}</span>
    </div>
  );
}

function SectionHeader({ children, action, help }) {
  return (
    <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", marginBottom: 10, paddingLeft: 2 }}>
      <span style={{ fontSize: T.fzFootnote, fontWeight: 600, color: T.labelPrimary, letterSpacing: T.trackJP, fontFamily: T.fontUI, display: "inline-flex", alignItems: "center", lineHeight: T.lhTight }}>
        {children}
        {help && <HelpButton help={help} />}
      </span>
      {action}
    </div>
  );
}

/* ═══════════════════════════════════════════
   HELP CONTENT (日本語)
   ═══════════════════════════════════════════ */
const HELP = {
  glyph:      { title: "字形", body: "解析する文字を1文字入力します。フォントに含まれていない文字は表示されません。" },
  size:       { title: "表示サイズ", body: "グリフを描画するピクセル高さ。曲率半径(px)もこのスケールに依存します。フォント間の比較は「半径 / em」を参照してください。" },
  changeFont: { title: "フォントを変更", body: "別の .ttf / .otf ファイルを読み込みます。" },

  comb:       { title: "曲率コーム", body: "輪郭の各サンプル点から法線方向に線を伸ばして曲率を可視化します。線が長いほど曲がりが急であることを示します。" },
  ctrl:       { title: "制御点", body: "ベジェ曲線のハンドル(制御点)。アンカーから伸びる細い線でアンカーとの関係を表示します。" },
  inflection: { title: "変曲点 (κ=0)", body: "曲率の符号が反転する点。凸と凹が切り替わる位置で、輪郭の S 字部分を特定できます。" },
  extrema:    { title: "曲率の極値", body: "曲率が局所的に最大または最小になる点。「角の丸み」がもっとも顕著に変化する箇所です。" },
  combScale:  { title: "コームの長さ", body: "コーム線の最大表示長を調整します。値を大きくすると小さな曲率変化も見やすくなります。" },

  segIdx:     { title: "セグメント番号", body: "輪郭を構成する個々のセグメントの通し番号(0始まり)。" },
  t:          { title: "パラメータ t", body: "ベジェセグメント上の位置を表す 0〜1 の値。0 が始点、1 が終点です。" },
  kappa:      { title: "曲率 κ", body: "その点での曲がり具合。値が大きいほど急なカーブです。符号は曲がる向きを示します。" },
  radius:     { title: "曲率半径", body: "R = 1 / |κ|。その点に最もよく当てはまる「接触円」の半径(ピクセル単位)。" },
  radiusEm:   { title: "半径 / em", body: "曲率半径を em スクエア単位で表したもの。フォントサイズや表示倍率に依存せず比較できます。" },
  theta:      { title: "接線角度 θ", body: "その点での接線の方向(ラジアン)。輪郭が進んでいる方向を示します。" },

  pinned:     { title: "固定された接触円", body: "輪郭上の任意の点をクリックすると、その位置の接触円を固定表示します。複数のグリフで形状の均質性を比較するときに便利です。" },

  segCount:   { title: "セグメント総数", body: "輪郭全体を構成するベジェ曲線および直線セグメントの数。" },
  cubic:      { title: "3次ベジェ", body: "4点で定義されるベジェ曲線。OpenType CFF (.otf) 系で多く使われます。" },
  quad:       { title: "2次ベジェ", body: "3点で定義されるベジェ曲線。TrueType (.ttf) で使われます。" },
  line:       { title: "直線", body: "2点を結ぶ線分。輪郭の直線部分の数です。" },
  maxK:       { title: "最大 |κ|", body: "グリフ内でもっとも大きな曲率の絶対値。デザインの「鋭さ」の指標です。" },
  minR:       { title: "最小半径", body: "最大曲率の逆数。輪郭中もっとも鋭い角の半径(px)。" },
  minRem:     { title: "最小半径 / em", body: "最小半径を em で正規化した値。異なるフォント・サイズ間での比較に適します。" },
  upm:        { title: "Units per em", body: "フォント内部の座標解像度。TrueType は 1024 / 2048、CFF は 1000 が一般的です。" },
};

/* ═══════════════════════════════════════════
   HELP BUTTON / POPOVER (Apple HIG: Offering help)
   ═══════════════════════════════════════════ */
function HelpButton({ help }) {
  const [open, setOpen] = useState(false);
  const [pos, setPos] = useState({ top: 0, left: 0 });
  const ref = useRef(null);
  const popRef = useRef(null);

  useEffect(() => {
    if (!open) return;
    const onDown = (e) => {
      if (ref.current && !ref.current.contains(e.target) &&
          popRef.current && !popRef.current.contains(e.target)) {
        setOpen(false);
      }
    };
    const onKey = (e) => { if (e.key === "Escape") setOpen(false); };
    document.addEventListener("mousedown", onDown);
    document.addEventListener("keydown", onKey);
    return () => {
      document.removeEventListener("mousedown", onDown);
      document.removeEventListener("keydown", onKey);
    };
  }, [open]);

  useEffect(() => {
    if (!open || !ref.current) return;
    const r = ref.current.getBoundingClientRect();
    const W = 280, H = 140;
    let top = r.bottom + 8;
    let left = r.left + r.width / 2 - W / 2;
    if (top + H > window.innerHeight - 12) top = r.top - 8 - H;
    if (left < 8) left = 8;
    if (left + W > window.innerWidth - 8) left = window.innerWidth - W - 8;
    setPos({ top, left });
  }, [open]);

  if (!help) return null;

  return (
    <>
      <button
        ref={ref}
        type="button"
        onClick={(e) => { e.stopPropagation(); e.preventDefault(); setOpen(o => !o); }}
        aria-label={`${help.title}の説明`}
        style={{
          width: 16, height: 16,
          padding: 0,
          borderRadius: "50%",
          border: "none",
          background: open ? T.accent : "rgba(120,120,128,0.40)",
          color: "#fff",
          fontSize: 11,
          fontWeight: 700,
          fontFamily: T.fontUI,
          cursor: "pointer",
          display: "inline-flex", alignItems: "center", justifyContent: "center",
          lineHeight: 1,
          marginLeft: 6,
          flexShrink: 0,
          transition: "background 0.15s",
          verticalAlign: "middle",
        }}
      >?</button>
      {open && (
        <div
          ref={popRef}
          role="tooltip"
          style={{
            position: "fixed",
            top: pos.top, left: pos.left,
            width: 280,
            background: "rgba(50,50,52,0.98)",
            backdropFilter: "saturate(180%) blur(20px)",
            WebkitBackdropFilter: "saturate(180%) blur(20px)",
            border: `1px solid ${T.border}`,
            borderRadius: T.radiusCard,
            padding: "14px 16px",
            boxShadow: "0 12px 32px rgba(0,0,0,0.5), 0 2px 8px rgba(0,0,0,0.3), inset 0 1px 0 rgba(255,255,255,0.08)",
            zIndex: 1000,
            fontFamily: T.fontUI,
          }}
          onClick={(e) => e.stopPropagation()}
        >
          <div style={{ fontSize: T.fzCallout, fontWeight: 700, color: T.labelPrimary, marginBottom: 8, letterSpacing: T.trackJP, lineHeight: T.lhTight }}>
            {help.title}
          </div>
          <div style={{ fontSize: T.fzFootnote, color: T.labelPrimary, lineHeight: T.lhBody, letterSpacing: T.trackJP }}>
            {help.body}
          </div>
        </div>
      )}
    </>
  );
}

function Toggle({ on, onChange }) {
  return (
    <button
      type="button"
      onClick={onChange}
      style={{
        width: 32, height: 18,
        borderRadius: 9,
        border: "none",
        background: on ? T.green : "rgba(120,120,128,0.32)",
        position: "relative",
        cursor: "pointer",
        transition: "background 0.18s",
        padding: 0,
        flexShrink: 0,
      }}
      aria-pressed={on}
    >
      <span style={{
        position: "absolute",
        top: 1, left: on ? 15 : 1,
        width: 16, height: 16,
        borderRadius: "50%",
        background: "#fff",
        boxShadow: "0 1px 2px rgba(0,0,0,0.4), 0 1px 3px rgba(0,0,0,0.15)",
        transition: "left 0.18s",
      }}/>
    </button>
  );
}

function Card({ children, accent, style }) {
  return (
    <div style={{
      background: T.bgGroup,
      borderRadius: T.radiusCard,
      padding: "10px 12px",
      border: `1px solid ${T.separator}`,
      borderLeft: accent ? `3px solid ${accent}` : `1px solid ${T.separator}`,
      ...style,
    }}>{children}</div>
  );
}

export default function GlyphAnalyzer() {
  const [otReady, setOtReady] = useState(false);
  const [font, setFont] = useState(null);
  const [fontName, setFontName] = useState("");
  const [char, setChar] = useState("a");
  const [displaySize, setDisplaySize] = useState(380);
  const [showComb, setShowComb] = useState(true);
  const [combScale, setCombScale] = useState(40);
  const [showCtrl, setShowCtrl] = useState(true);
  const [showInflections, setShowInflections] = useState(true);
  const [showExtrema, setShowExtrema] = useState(true);
  const [pinnedOsc, setPinnedOsc] = useState(null);
  const [hoverInfo, setHoverInfo] = useState(null);
  const [hoveredSeg, setHoveredSeg] = useState(null);
  const svgRef = useRef(null);
  const canvasW = 560;
  const canvasH = 520;
  const plotH = 140;

  // opentype.js is now bundled via import — no CDN load needed
  useEffect(() => { setOtReady(true); }, []);

  const handleFile = useCallback(async (file) => {
    try {
      const buf = await file.arrayBuffer();
      const f = opentype.parse(buf);
      setFont(f);
      setFontName(f.names?.fontFamily?.en || f.names?.fontFamily || file.name);
    } catch (e) {
      alert("Failed to parse font: " + e.message);
    }
  }, []);

  const handleDrop = useCallback((e) => {
    e.preventDefault();
    const file = e.dataTransfer?.files?.[0];
    if (file) handleFile(file);
  }, [handleFile]);

  // Parse glyph
  const { pathD, segments, analyzed, unitsPerEm, glyphWidth } = useMemo(() => {
    if (!font || !char) return { pathD: "", segments: [], analyzed: [], unitsPerEm: 1000, glyphWidth: 0 };
    const glyph = font.charToGlyph(char);
    if (!glyph) return { pathD: "", segments: [], analyzed: [], unitsPerEm: 1000, glyphWidth: 0 };
    const upm = font.unitsPerEm || 1000;
    const scale = displaySize / upm;
    const xOff = (canvasW - glyph.advanceWidth * scale) / 2;
    const yOff = canvasH * 0.78;
    const path = glyph.getPath(xOff, yOff, displaySize);
    const segs = parseGlyphPath(path.commands);
    const anal = analyzeSegments(segs);
    return { pathD: path.toPathData(4), segments: segs, analyzed: anal, unitsPerEm: upm, glyphWidth: glyph.advanceWidth };
  }, [font, char, displaySize, canvasW, canvasH]);

  // Global max curvature for normalization
  const maxK = useMemo(() => {
    let mk = 0;
    analyzed.forEach(seg => seg.samples.forEach(s => { if (Math.abs(s.k) > mk) mk = Math.abs(s.k); }));
    return mk || 1;
  }, [analyzed]);

  // Handle hover on the outline
  const handleSvgMove = useCallback((e) => {
    if (!svgRef.current || analyzed.length === 0) return;
    const rect = svgRef.current.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    let best = null;
    let bestDist = 12;
    analyzed.forEach((seg, si) => {
      seg.samples.forEach((s) => {
        const d = Math.sqrt((s.x - mx)**2 + (s.y - my)**2);
        if (d < bestDist) { bestDist = d; best = { segIdx: si, t: s.t, ...s }; }
      });
    });
    if (best) {
      const e2 = evalSegment(analyzed[best.segIdx], best.t);
      const osc = osculatingCircle(e2, e2.d1, e2.k);
      setHoverInfo({ ...best, osc, segIdx: best.segIdx });
      setHoveredSeg(best.segIdx);
    } else {
      setHoverInfo(null);
      setHoveredSeg(null);
    }
  }, [analyzed]);

  const handleSvgClick = useCallback(() => {
    if (hoverInfo) setPinnedOsc({ segIdx: hoverInfo.segIdx, t: hoverInfo.t });
  }, [hoverInfo]);

  // Pinned osculating circle data
  const pinnedData = useMemo(() => {
    if (!pinnedOsc || !analyzed[pinnedOsc.segIdx]) return null;
    const e = evalSegment(analyzed[pinnedOsc.segIdx], pinnedOsc.t);
    const osc = osculatingCircle(e, e.d1, e.k);
    return { ...e, osc, segIdx: pinnedOsc.segIdx, t: pinnedOsc.t };
  }, [pinnedOsc, analyzed]);

  const emScale = displaySize;

  // All inflection points
  const allInflections = useMemo(() => {
    const pts = [];
    analyzed.forEach((seg, si) => {
      seg.inflections.forEach(t => {
        const e = evalSegment(seg, t);
        pts.push({ ...e, segIdx: si, t });
      });
    });
    return pts;
  }, [analyzed]);

  // All extrema
  const allExtrema = useMemo(() => {
    const pts = [];
    analyzed.forEach((seg, si) => {
      seg.extrema.forEach(ex => {
        const e = evalSegment(seg, ex.t);
        pts.push({ ...e, segIdx: si, t: ex.t });
      });
    });
    return pts;
  }, [analyzed]);

  // file input ref
  const fileRef = useRef(null);

  const baseY = canvasH * 0.78;

  return (
    <div style={{ minHeight: "100vh", background: T.bgWindow, color: T.labelPrimary, fontFamily: T.fontUI, WebkitFontSmoothing: "antialiased" }}>
      {/* Toolbar (macOS-style) */}
      <div style={{
        padding: "0 24px",
        height: 56,
        borderBottom: `1px solid ${T.separator}`,
        display: "flex",
        alignItems: "center",
        gap: 18,
        background: T.bgToolbar,
        backdropFilter: "saturate(180%) blur(20px)",
        WebkitBackdropFilter: "saturate(180%) blur(20px)",
        position: "sticky", top: 0, zIndex: 10,
      }}>
        <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
          <div style={{ width: 24, height: 24, borderRadius: 6, background: `linear-gradient(135deg, ${T.accent}, ${T.teal})`, display: "flex", alignItems: "center", justifyContent: "center", fontSize: 14, color: "#fff", fontWeight: 700 }}>𝛋</div>
          <span style={{ fontSize: T.fzCallout, fontWeight: 600, color: T.labelPrimary, letterSpacing: T.trackJP }}>Graphicurve</span>
        </div>

        <div style={{ width: 1, height: 24, background: T.separator }} />

        {!font ? (
          <button
            onDragOver={(e) => e.preventDefault()}
            onDrop={handleDrop}
            onClick={() => fileRef.current?.click()}
            disabled={!otReady}
            style={{
              padding: "7px 14px",
              border: "none",
              borderRadius: T.radiusControl,
              cursor: otReady ? "pointer" : "wait",
              fontSize: T.fzFootnote,
              fontWeight: 600,
              color: "#fff",
              background: otReady ? T.accent : T.bgGroup,
              fontFamily: T.fontUI,
              letterSpacing: T.trackJP,
              transition: "background 0.15s",
            }}
          >
            {otReady ? "フォントを開く…" : "読み込み中…"}
          </button>
        ) : (
          <span style={{ fontSize: T.fzFootnote, color: T.labelPrimary, fontWeight: 500, letterSpacing: T.trackJP }}>{fontName}</span>
        )}
        <input ref={fileRef} type="file" accept=".ttf,.otf,.woff" style={{ display: "none" }}
          onChange={(e) => { if (e.target.files[0]) handleFile(e.target.files[0]); }} />

        {font && (
          <>
            <div style={{ width: 1, height: 24, background: T.separator }} />

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label style={{ fontSize: T.fzFootnote, color: T.labelSecondary, display: "inline-flex", alignItems: "center", whiteSpace: "nowrap", letterSpacing: T.trackJP }}>
                字形<HelpButton help={HELP.glyph}/>
              </label>
              <input
                value={char}
                onKeyDown={(e) => {
                  if (e.key.length === 1) {
                    setChar(e.key);
                    e.preventDefault();
                  }
                }}
                onChange={() => {}}
                onFocus={(e) => e.target.select()}
                style={{
                  width: 40, height: 28, textAlign: "center",
                  background: T.bgField,
                  border: `1px solid ${T.border}`,
                  borderRadius: T.radiusControl,
                  color: T.labelPrimary,
                  fontSize: T.fzHeadline, fontWeight: 500,
                  fontFamily: T.fontUI, outline: "none",
                }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
              <label style={{ fontSize: T.fzFootnote, color: T.labelSecondary, display: "inline-flex", alignItems: "center", whiteSpace: "nowrap", letterSpacing: T.trackJP }}>
                サイズ<HelpButton help={HELP.size}/>
              </label>
              <input type="range" min={150} max={480} value={displaySize} onChange={(e) => setDisplaySize(Number(e.target.value))}
                style={{ width: 100, accentColor: T.accent }} />
              <span style={{ fontSize: T.fzCaption, color: T.labelSecondary, fontFamily: T.fontMono, minWidth: 32 }}>{displaySize}</span>
            </div>

            <div style={{ flex: 1 }} />

            <button onClick={() => { setFont(null); setFontName(""); setPinnedOsc(null); }}
              style={{
                background: T.bgGroup,
                border: `1px solid ${T.separator}`,
                borderRadius: T.radiusControl,
                color: T.labelPrimary,
                cursor: "pointer",
                padding: "6px 12px",
                fontSize: T.fzFootnote,
                fontFamily: T.fontUI,
                fontWeight: 500,
                letterSpacing: T.trackJP,
              }}>
              フォントを変更
            </button>
          </>
        )}
      </div>

      {!font ? (
        <div style={{ display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", height: "calc(100vh - 56px)", gap: 28 }}
          onDragOver={(e) => e.preventDefault()} onDrop={handleDrop}>
          <div style={{
            width: 104, height: 104, borderRadius: 24,
            background: `linear-gradient(135deg, ${T.accent}, ${T.teal})`,
            display: "flex", alignItems: "center", justifyContent: "center",
            fontSize: 48, color: "#fff", fontWeight: 700,
            boxShadow: "0 12px 32px rgba(10,132,255,0.25), inset 0 1px 0 rgba(255,255,255,0.2)",
          }}>𝛋</div>
          <div style={{ textAlign: "center", maxWidth: 420, padding: "0 24px" }}>
            <div style={{ fontSize: 24, fontWeight: 700, color: T.labelPrimary, letterSpacing: T.trackJP, marginBottom: 12, lineHeight: T.lhTight }}>
              フォントを開く
            </div>
            <div style={{ fontSize: T.fzBody, color: T.labelSecondary, lineHeight: T.lhBody, letterSpacing: T.trackJP }}>
              .ttf または .otf ファイルをドラッグ&ドロップ、<br/>または上の「フォントを開く…」ボタンから選択してください。
            </div>
            <div style={{ fontSize: T.fzFootnote, color: T.labelTertiary, marginTop: 16, lineHeight: T.lhBody, letterSpacing: T.trackJP }}>
              Google Fonts から Jost、Montserrat などをダウンロードして使えます
            </div>
          </div>
        </div>
      ) : (
        <div style={{ display: "flex", flexWrap: "wrap", padding: 20, gap: 20 }}>
          {/* Main canvas */}
          <div style={{ flex: "1 1 560px" }}>
            <svg
              ref={svgRef}
              width={canvasW} height={canvasH}
              style={{ background: T.bgContent, borderRadius: T.radiusCard, border: `1px solid ${T.separator}`, display: "block", cursor: "crosshair", boxShadow: "0 1px 0 rgba(255,255,255,0.04) inset" }}
              onMouseMove={handleSvgMove}
              onMouseLeave={() => { setHoverInfo(null); setHoveredSeg(null); }}
              onClick={handleSvgClick}
            >
              {/* Grid */}
              <defs>
                <pattern id="gg" width={displaySize/8} height={displaySize/8} patternUnits="userSpaceOnUse">
                  <path d={`M ${displaySize/8} 0 L 0 0 0 ${displaySize/8}`} fill="none" stroke="#151515" strokeWidth="0.5"/>
                </pattern>
              </defs>
              <rect width={canvasW} height={canvasH} fill="url(#gg)"/>

              {/* Guide lines */}
              <line x1={0} y1={baseY} x2={canvasW} y2={baseY} stroke="#222" strokeWidth={0.5} strokeDasharray="3,3"/>
              <text x={6} y={baseY-3} fill="#2a2a2a" fontSize={9} fontFamily="inherit">ベースライン</text>

              {/* Glyph fill (faint) */}
              <path d={pathD} fill="rgba(255,255,255,0.04)" stroke="none"/>

              {/* Curvature comb */}
              {showComb && analyzed.map((seg, si) => (
                <g key={`comb-${si}`}>
                  {seg.samples.filter((_, i) => i % 2 === 0).map((s, i) => {
                    if (Math.abs(s.k) < 1e-8) return null;
                    const speed = Math.sqrt(s.d1.x**2 + s.d1.y**2);
                    if (speed < 1e-8) return null;
                    const nx = -s.d1.y / speed;
                    const ny = s.d1.x / speed;
                    const sign = s.k > 0 ? 1 : -1;
                    const len = Math.min(combScale * Math.abs(s.k) / maxK, combScale);
                    return (
                      <line key={i} x1={s.x} y1={s.y} x2={s.x + sign*nx*len} y2={s.y + sign*ny*len}
                        stroke={kColor(s.k, maxK)} strokeWidth={0.7} opacity={0.5}/>
                    );
                  })}
                </g>
              ))}

              {/* Outline segments (color-coded) */}
              {analyzed.map((seg, si) => {
                const pts = seg.samples;
                let d = `M ${pts[0].x} ${pts[0].y}`;
                for (let i = 1; i < pts.length; i++) d += ` L ${pts[i].x} ${pts[i].y}`;
                const isHov = hoveredSeg === si;
                return (
                  <path key={`seg-${si}`} d={d} fill="none"
                    stroke={isHov ? "#fff" : kColor(seg.samples[Math.floor(seg.samples.length/2)].k, maxK)}
                    strokeWidth={isHov ? 2.2 : 1.5} opacity={isHov ? 1 : 0.85}/>
                );
              })}

              {/* Control points */}
              {showCtrl && analyzed.map((seg, si) => {
                if (seg.type === "line") return null;
                const pts = seg.type === "cubic"
                  ? [{ from: seg.p0, to: seg.p1 }, { from: seg.p3, to: seg.p2 }]
                  : [{ from: seg.p0, to: seg.p1 }, { from: seg.p2, to: seg.p1 }];
                return (
                  <g key={`ctrl-${si}`}>
                    {pts.map((p, i) => (
                      <g key={i}>
                        <line x1={p.from.x} y1={p.from.y} x2={p.to.x} y2={p.to.y} stroke="#333" strokeWidth={0.5}/>
                        <circle cx={p.to.x} cy={p.to.y} r={2.5} fill="none" stroke="#555" strokeWidth={0.8}/>
                      </g>
                    ))}
                  </g>
                );
              })}

              {/* Inflection points */}
              {showInflections && allInflections.map((pt, i) => (
                <g key={`inf-${i}`}>
                  <circle cx={pt.x} cy={pt.y} r={4.5} fill="none" stroke={T.red} strokeWidth={1.5}/>
                  <circle cx={pt.x} cy={pt.y} r={1.5} fill={T.red}/>
                </g>
              ))}

              {/* Curvature extrema */}
              {showExtrema && allExtrema.map((pt, i) => (
                <g key={`ext-${i}`}>
                  <rect x={pt.x-3.5} y={pt.y-3.5} width={7} height={7} fill="none" stroke={T.yellow} strokeWidth={1.2} transform={`rotate(45,${pt.x},${pt.y})`}/>
                </g>
              ))}

              {/* Pinned osculating circle */}
              {pinnedData?.osc && (
                <g>
                  <circle cx={pinnedData.osc.cx} cy={pinnedData.osc.cy} r={pinnedData.osc.r}
                    fill="none" stroke={T.green} strokeWidth={1.2} strokeDasharray="4,3" opacity={0.7}/>
                  <line x1={pinnedData.x} y1={pinnedData.y} x2={pinnedData.osc.cx} y2={pinnedData.osc.cy}
                    stroke={T.green} strokeWidth={0.6} strokeDasharray="2,2" opacity={0.5}/>
                  <circle cx={pinnedData.osc.cx} cy={pinnedData.osc.cy} r={2} fill={T.green}/>
                  <circle cx={pinnedData.x} cy={pinnedData.y} r={3} fill={T.green}/>
                  <text x={pinnedData.osc.cx + 8} y={pinnedData.osc.cy - 6} fill={T.green} fontSize={10} fontFamily="inherit">
                    R={pinnedData.osc.r.toFixed(1)}px ({(pinnedData.osc.r/emScale).toFixed(3)}em)
                  </text>
                </g>
              )}

              {/* Hover osculating circle */}
              {hoverInfo?.osc && (!pinnedData || hoverInfo.segIdx !== pinnedData.segIdx || Math.abs(hoverInfo.t - pinnedData.t) > 0.02) && (
                <g>
                  <circle cx={hoverInfo.osc.cx} cy={hoverInfo.osc.cy} r={Math.min(hoverInfo.osc.r, 300)}
                    fill="none" stroke="#fff" strokeWidth={0.8} strokeDasharray="3,3" opacity={0.3}/>
                  <circle cx={hoverInfo.x} cy={hoverInfo.y} r={3} fill="#fff" opacity={0.6}/>
                </g>
              )}

              {/* Legend */}
              <g transform="translate(16, 22)" fontFamily={T.fontUI}>
                {showInflections && <><circle cx={5} cy={0} r={4} fill="none" stroke={T.red} strokeWidth={1.4}/><text x={16} y={4} fill={T.red} fontSize={12} fontWeight={500}>変曲点 (κ=0)</text></>}
                {showExtrema && <><rect x={1} y={15} width={8} height={8} fill="none" stroke={T.yellow} strokeWidth={1.2} transform="rotate(45,5,19)"/><text x={16} y={23} fill={T.yellow} fontSize={12} fontWeight={500}>曲率の極値</text></>}
                <text x={0} y={42} fill={T.green} fontSize={12} fontWeight={500}>輪郭をクリック → 接触円を固定</text>
              </g>
            </svg>

            {/* Curvature plot */}
            {analyzed.length > 0 && (
              <div style={{ marginTop: 16 }}>
                <div style={{ fontSize: T.fzFootnote, color: T.labelPrimary, marginBottom: 10, fontWeight: 600, display: "inline-flex", alignItems: "center", letterSpacing: T.trackJP }}>
                  経路に沿った曲率 κ(t)<HelpButton help={HELP.kappa}/>
                </div>
                <svg width={canvasW} height={plotH} style={{ background: T.bgContent, borderRadius: T.radiusCard, border: `1px solid ${T.separator}`, display: "block" }}>
                  {/* zero line */}
                  <line x1={0} y1={plotH/2} x2={canvasW} y2={plotH/2} stroke="#1a1a1a" strokeWidth={0.5}/>
                  <text x={4} y={plotH/2 - 3} fill="#2a2a2a" fontSize={8}>κ=0</text>

                  {(() => {
                    const totalSamples = analyzed.reduce((s, seg) => s + seg.samples.length, 0);
                    let idx = 0;
                    const scaleY = (plotH / 2 - 10) / maxK;
                    return analyzed.map((seg, si) => {
                      const startIdx = idx;
                      const points = seg.samples.map((s, i) => {
                        const x = ((startIdx + i) / totalSamples) * canvasW;
                        const y = plotH / 2 - s.k * scaleY;
                        return `${x},${y}`;
                      });
                      idx += seg.samples.length;
                      // segment separators
                      const sepX = (startIdx / totalSamples) * canvasW;
                      return (
                        <g key={si}>
                          {si > 0 && <line x1={sepX} y1={0} x2={sepX} y2={plotH} stroke="#1a1a1a" strokeWidth={0.5}/>}
                          <polyline points={points.join(" ")} fill="none"
                            stroke={hoveredSeg === si ? "#fff" : kColor(seg.samples[Math.floor(seg.samples.length/2)].k, maxK)}
                            strokeWidth={hoveredSeg === si ? 1.5 : 1} opacity={0.8}/>
                          {/* inflections */}
                          {seg.inflections.map((t, ii) => {
                            const ix = ((startIdx + t * seg.samples.length) / totalSamples) * canvasW;
                            return <circle key={ii} cx={ix} cy={plotH/2} r={3} fill={T.red} opacity={0.7}/>;
                          })}
                        </g>
                      );
                    });
                  })()}

                  {/* Hover marker on plot */}
                  {hoverInfo && (() => {
                    let totalBefore = 0;
                    for (let i = 0; i < hoverInfo.segIdx; i++) totalBefore += analyzed[i].samples.length;
                    const total = analyzed.reduce((s, seg) => s + seg.samples.length, 0);
                    const approxIdx = totalBefore + hoverInfo.t * analyzed[hoverInfo.segIdx].samples.length;
                    const hx = (approxIdx / total) * canvasW;
                    const scaleY = (plotH / 2 - 10) / maxK;
                    const hy = plotH / 2 - hoverInfo.k * scaleY;
                    return (
                      <g>
                        <line x1={hx} y1={0} x2={hx} y2={plotH} stroke="#fff" strokeWidth={0.5} opacity={0.3}/>
                        <circle cx={hx} cy={hy} r={3} fill="#fff"/>
                      </g>
                    );
                  })()}
                </svg>
              </div>
            )}
          </div>

          {/* Right inspector panel (Xcode-style) */}
          <div style={{
            width: 280,
            flexShrink: 0,
            display: "flex", flexDirection: "column", gap: 18,
            fontSize: T.fzFootnote, lineHeight: T.lhBody,
          }}>
            {/* Display toggles */}
            <div>
              <SectionHeader>表示</SectionHeader>
              <Card>
                {[
                  ["曲率コーム", showComb, setShowComb, HELP.comb],
                  ["制御点", showCtrl, setShowCtrl, HELP.ctrl],
                  ["変曲点", showInflections, setShowInflections, HELP.inflection],
                  ["曲率の極値", showExtrema, setShowExtrema, HELP.extrema],
                ].map(([label, val, setter, help]) => (
                  <div key={label} style={{
                    display: "flex", alignItems: "center", justifyContent: "space-between",
                    padding: "6px 0",
                    color: T.labelPrimary, fontSize: T.fzFootnote, letterSpacing: T.trackJP,
                  }}>
                    <span style={{ display: "inline-flex", alignItems: "center", cursor: "pointer" }} onClick={() => setter(!val)}>
                      {label}
                    </span>
                    <span style={{ display: "inline-flex", alignItems: "center", gap: 8 }}>
                      <HelpButton help={help} />
                      <Toggle on={val} onChange={() => setter(!val)} />
                    </span>
                  </div>
                ))}
                {showComb && (
                  <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 10, paddingTop: 10, borderTop: `1px solid ${T.separator}` }}>
                    <span style={{ fontSize: T.fzFootnote, color: T.labelSecondary, flex: 1, display: "inline-flex", alignItems: "center", letterSpacing: T.trackJP }}>
                      コームの長さ<HelpButton help={HELP.combScale}/>
                    </span>
                    <input type="range" min={10} max={100} value={combScale} onChange={(e) => setCombScale(Number(e.target.value))}
                      style={{ width: 80, accentColor: T.accent }}/>
                    <span style={{ fontSize: T.fzCaption, color: T.labelSecondary, fontFamily: T.fontMono, minWidth: 24, textAlign: "right" }}>{combScale}</span>
                  </div>
                )}
              </Card>
            </div>

            {/* Hover info */}
            <div>
              <SectionHeader>{hoverInfo ? "ポイント" : "インスペクタ"}</SectionHeader>
              {hoverInfo ? (
                <Card>
                  <Row label="セグメント" value={`#${hoverInfo.segIdx}`} help={HELP.segIdx}/>
                  <Row label="t" value={hoverInfo.t.toFixed(3)} help={HELP.t}/>
                  <Row label="κ" value={hoverInfo.k.toFixed(5)} help={HELP.kappa}/>
                  <Row label="曲率半径" value={Math.abs(hoverInfo.k) > 1e-8 ? `${(1/Math.abs(hoverInfo.k)).toFixed(1)} px` : "∞"} help={HELP.radius}/>
                  <Row label="半径 / em" value={Math.abs(hoverInfo.k) > 1e-8 ? (1/Math.abs(hoverInfo.k)/emScale).toFixed(4) : "∞"} help={HELP.radiusEm}/>
                  {hoverInfo.d1 && (
                    <Row label="接線角度 θ" value={`${Math.atan2(-hoverInfo.d1.y, hoverInfo.d1.x).toFixed(3)} rad`} help={HELP.theta}/>
                  )}
                </Card>
              ) : (
                <Card>
                  <div style={{ color: T.labelSecondary, fontSize: T.fzFootnote, padding: "6px 0", textAlign: "center", letterSpacing: T.trackJP, lineHeight: T.lhBody }}>
                    輪郭にカーソルを合わせて確認
                  </div>
                </Card>
              )}
            </div>

            {/* Pinned */}
            {pinnedData && (
              <div>
                <SectionHeader help={HELP.pinned} action={
                  <button onClick={() => setPinnedOsc(null)} title="固定を解除"
                    style={{
                      background: "transparent", border: "none",
                      color: T.labelTertiary, cursor: "pointer",
                      padding: 0, fontSize: 14, lineHeight: 1, width: 16, height: 16,
                    }}>×</button>
                }>固定</SectionHeader>
                <Card accent={T.green}>
                  <Row label="セグメント" value={`#${pinnedData.segIdx}`} help={HELP.segIdx}/>
                  <Row label="t" value={pinnedData.t.toFixed(3)} help={HELP.t}/>
                  <Row label="κ" value={pinnedData.k.toFixed(5)} help={HELP.kappa}/>
                  <Row label="曲率半径" value={pinnedData.osc ? `${pinnedData.osc.r.toFixed(1)} px` : "∞"} help={HELP.radius}/>
                  <Row label="半径 / em" value={pinnedData.osc ? (pinnedData.osc.r/emScale).toFixed(4) : "∞"} help={HELP.radiusEm}/>
                </Card>
              </div>
            )}

            {/* Summary */}
            <div>
              <SectionHeader>サマリー</SectionHeader>
              <Card>
                <Row label="セグメント数" value={analyzed.length} help={HELP.segCount}/>
                <Row label="3次ベジェ" value={analyzed.filter(s => s.type==="cubic").length} help={HELP.cubic}/>
                <Row label="2次ベジェ" value={analyzed.filter(s => s.type==="quad").length} help={HELP.quad}/>
                <Row label="直線" value={analyzed.filter(s => s.type==="line").length} help={HELP.line}/>
                <div style={{ height: 1, background: T.separator, margin: "6px 0" }} />
                <Row label="変曲点" value={allInflections.length} help={HELP.inflection}/>
                <Row label="曲率の極値" value={allExtrema.length} help={HELP.extrema}/>
                <Row label="最大 |κ|" value={maxK.toFixed(4)} help={HELP.maxK}/>
                <Row label="最小半径" value={maxK > 1e-8 ? `${(1/maxK).toFixed(1)} px` : "∞"} help={HELP.minR}/>
                <Row label="最小半径 / em" value={maxK > 1e-8 ? (1/maxK/emScale).toFixed(4) : "∞"} help={HELP.minRem}/>
                {font && <Row label="em単位数" value={font.unitsPerEm} help={HELP.upm}/>}
              </Card>
            </div>

            {/* Segment list */}
            <div>
              <SectionHeader>セグメント一覧</SectionHeader>
              <Card style={{ padding: 4 }}>
                <div style={{ maxHeight: 240, overflowY: "auto" }}>
                  {analyzed.map((seg, si) => {
                    const midK = seg.samples[Math.floor(seg.samples.length/2)]?.k || 0;
                    const isHov = hoveredSeg === si;
                    return (
                      <div key={si}
                        onMouseEnter={() => setHoveredSeg(si)}
                        onMouseLeave={() => setHoveredSeg(null)}
                        style={{
                          padding: "6px 8px",
                          borderRadius: T.radiusInline,
                          marginBottom: 1,
                          cursor: "default",
                          background: isHov ? "rgba(10,132,255,0.15)" : "transparent",
                          display: "flex", alignItems: "center", gap: 10,
                          fontSize: T.fzFootnote,
                          letterSpacing: T.trackJP,
                        }}>
                        <div style={{ width: 10, height: 10, borderRadius: 3, background: kColor(midK, maxK), flexShrink: 0, boxShadow: "inset 0 0 0 1px rgba(0,0,0,0.3)" }}/>
                        <span style={{ color: T.labelSecondary, fontFamily: T.fontMono, minWidth: 28, fontSize: T.fzCaption }}>#{si}</span>
                        <span style={{ color: T.labelPrimary, flex: 1 }}>{ {cubic: "3次ベジェ", quad: "2次ベジェ", line: "直線"}[seg.type] || seg.type }</span>
                        {seg.inflections.length > 0 && (
                          <span style={{
                            color: T.red, fontSize: T.fzCaption, fontFamily: T.fontUI,
                            background: "rgba(255,69,58,0.16)", padding: "2px 7px", borderRadius: 4,
                            letterSpacing: T.trackJP, fontWeight: 500,
                          }}>変曲 {seg.inflections.length}</span>
                        )}
                      </div>
                    );
                  })}
                </div>
              </Card>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
