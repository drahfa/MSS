import fs from 'node:fs';
import path from 'node:path';
import { execSync } from 'node:child_process';
import { marked } from 'marked';
import katex from 'katex';

const toPublishDir = path.resolve('..', 'to-publish');
const chromePath = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';

const papers = [
  'IEEE_TECHNICAL_REPORT-Paper-1',
  'IEEE_TRAINING_PLATFORM_REPORT-Paper-2',
  'IEEE_CABLE_NAVIGATION_REPORT-Paper-3',
  'IEEE_UVMS_MANIPULATION_REPORT-Paper-4',
  'IEEE_MULTI_ROV_NETWORKED_REPORT-Paper-5',
  'IEEE_SONAR_PERCEPTION_REPORT-Paper-6',
];

// Read KaTeX CSS
const katexCssPath = path.resolve('node_modules', 'katex', 'dist', 'katex.min.css');
const katexCss = fs.readFileSync(katexCssPath, 'utf8');

const ieeeStyles = `
  @page {
    size: letter;
    margin: 20mm 18mm 22mm 18mm;
  }
  @media print {
    body {
      -webkit-print-color-adjust: exact;
      print-color-adjust: exact;
    }
    .page-break {
      page-break-before: always;
    }
  }
  * {
    box-sizing: border-box;
  }
  body {
    font-family: "Times New Roman", Times, "Nimbus Roman No9 L", serif;
    font-size: 10pt;
    line-height: 1.45;
    color: #111;
    background: #fff;
    margin: 0;
    padding: 0;
    text-align: justify;
  }
  h1 {
    font-size: 19pt;
    font-weight: bold;
    text-align: center;
    margin: 0 0 10px 0;
    line-height: 1.25;
    text-transform: none;
  }
  .authors-block {
    text-align: center;
    font-size: 10.5pt;
    font-style: italic;
    margin-bottom: 22px;
    line-height: 1.4;
  }
  .authors-block strong {
    font-style: normal;
    font-weight: 600;
  }
  h2 {
    font-size: 11pt;
    font-weight: bold;
    text-transform: uppercase;
    margin-top: 20px;
    margin-bottom: 8px;
    border-bottom: 1px solid #333;
    padding-bottom: 2px;
    page-break-after: avoid;
    letter-spacing: 0.05em;
  }
  h3 {
    font-size: 10pt;
    font-weight: bold;
    font-style: italic;
    margin-top: 14px;
    margin-bottom: 6px;
    page-break-after: avoid;
  }
  h4 {
    font-size: 9.5pt;
    font-weight: 600;
    margin-top: 10px;
    margin-bottom: 4px;
    page-break-after: avoid;
  }
  p {
    margin: 0 0 10px 0;
    text-indent: 14pt;
  }
  p.no-indent, .abstract-p, .keywords-p {
    text-indent: 0 !important;
  }
  .abstract-box {
    border-left: 3px solid #000;
    padding: 4px 12px;
    margin: 16px 0 20px 0;
    background: #fdfdfd;
    font-size: 9.5pt;
  }
  .abstract-p {
    font-weight: bold;
  }
  .keywords-p {
    margin-top: 6px;
    font-size: 9pt;
  }
  .keywords-label {
    font-weight: bold;
    font-style: italic;
  }
  /* KaTeX display container */
  .katex-display-container {
    text-align: center;
    margin: 10px 0;
    page-break-inside: avoid;
  }
  .katex-display {
    margin: 0.5em 0 !important;
  }
  /* Table styling */
  table {
    width: 100%;
    border-collapse: collapse;
    margin: 16px 0;
    font-size: 8.5pt;
    page-break-inside: avoid;
  }
  th, td {
    padding: 5px 8px;
    text-align: left;
    vertical-align: middle;
  }
  thead th {
    border-top: 2px solid #000;
    border-bottom: 1.5px solid #000;
    font-weight: bold;
    text-transform: uppercase;
    font-size: 8pt;
    letter-spacing: 0.03em;
  }
  tbody tr {
    border-bottom: 0.5px solid #ddd;
  }
  tbody tr:last-child td {
    border-bottom: 2px solid #000;
  }
  /* Code and blockquotes */
  pre, code {
    font-family: "Consolas", "Courier New", monospace;
  }
  pre {
    background: #f7f9fa;
    border: 1px solid #d8dee2;
    border-radius: 3px;
    padding: 10px 14px;
    font-size: 8pt;
    line-height: 1.35;
    overflow-x: auto;
    page-break-inside: avoid;
    margin: 12px 0;
  }
  code {
    background: #f0f2f4;
    padding: 1px 4px;
    border-radius: 2px;
    font-size: 8.5pt;
  }
  pre code {
    background: transparent;
    padding: 0;
  }
  blockquote {
    border-left: 3px solid #888;
    margin: 12px 0;
    padding: 4px 14px;
    color: #444;
    font-style: italic;
  }
  ul, ol {
    margin: 0 0 10px 0;
    padding-left: 24px;
  }
  li {
    margin-bottom: 4px;
  }
  hr {
    border: 0;
    border-top: 1px solid #ccc;
    margin: 18px 0;
  }
  a {
    color: #0366d6;
    text-decoration: none;
  }
`;

function renderMarkdownToHtml(markdownContent) {
  // Step 1: Protect display math $$ ... $$
  const displayMath = [];
  let processed = markdownContent.replace(/\$\$([\s\S]+?)\$\$/g, (match, math) => {
    const idx = displayMath.length;
    displayMath.push(math.trim());
    return `___DISPLAY_MATH_${idx}___`;
  });

  // Step 2: Protect inline math $ ... $ (excluding escaped \$)
  const inlineMath = [];
  processed = processed.replace(/(?<!\\)\$([^\$\n]+?)\$/g, (match, math) => {
    const idx = inlineMath.length;
    inlineMath.push(math.trim());
    return `___INLINE_MATH_${idx}___`;
  });

  // Step 3: Run marked parser
  let html = marked.parse(processed, { gfm: true, breaks: false });

  // Step 4: Re-insert rendered KaTeX math
  displayMath.forEach((math, idx) => {
    let rendered = '';
    try {
      rendered = katex.renderToString(math, { displayMode: true, throwOnError: false });
    } catch (e) {
      rendered = `<pre>${math}</pre>`;
    }
    const token = `___DISPLAY_MATH_${idx}___`;
    // If inside a <p>, replace outer <p> or wrap
    html = html.replace(`<p>${token}</p>`, `<div class="katex-display-container">${rendered}</div>`);
    html = html.replace(token, `<div class="katex-display-container">${rendered}</div>`);
  });

  inlineMath.forEach((math, idx) => {
    let rendered = '';
    try {
      rendered = katex.renderToString(math, { displayMode: false, throwOnError: false });
    } catch (e) {
      rendered = `<code>${math}</code>`;
    }
    const token = `___INLINE_MATH_${idx}___`;
    html = html.replaceAll(token, rendered);
  });

  return html;
}

for (const paperName of papers) {
  const mdPath = path.join(toPublishDir, `${paperName}.md`);
  const htmlPath = path.join(toPublishDir, `${paperName}.html`);
  const pdfPath = path.join(toPublishDir, `${paperName}.pdf`);

  if (!fs.existsSync(mdPath)) {
    console.error(`File not found: ${mdPath}`);
    continue;
  }

  console.log(`Processing: ${paperName}.md -> ${paperName}.pdf...`);
  const mdContent = fs.readFileSync(mdPath, 'utf8');
  const bodyHtml = renderMarkdownToHtml(mdContent);

  const fullHtml = `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>${paperName}</title>
  <style>
    ${katexCss}
    ${ieeeStyles}
  </style>
</head>
<body>
  ${bodyHtml}
</body>
</html>`;

  fs.writeFileSync(htmlPath, fullHtml, 'utf8');

  // Print to PDF via Chrome headless directly without cmd /c shell
  const chromeArgs = [
    '--headless=new',
    `--print-to-pdf=${pdfPath}`,
    '--no-pdf-header-footer',
    htmlPath,
  ];

  try {
    const { execFileSync } = await import('node:child_process');
    execFileSync(chromePath, chromeArgs, { stdio: 'inherit' });
    console.log(`✓ Generated PDF: ${pdfPath}`);
  } catch (err) {
    console.error(`Failed to generate PDF for ${paperName}:`, err.message);
  }
}

console.log('All PDF papers processed successfully!');
