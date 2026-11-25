// xacro features on in-memory files: everything the golden corpus cannot
// show (error locations, traces, file-system behaviour, warnings) plus a
// direct check of each directive.

import { test } from "node:test";
import assert from "node:assert/strict";
import { doc, expand, memFs } from "./helpers.js";
import {
  XacroError,
  expandXacro,
  looksLikeXacro,
  parseDocument,
  parseMacroArg,
  serializeXml,
  toXmlElement,
  XmlSyntaxError,
} from "../src/core/xacro/index.js";
import { cloneNode } from "../src/core/xacro/dom.js";
import { evalLiteral } from "../src/core/xacro/table.js";

/** @param {() => unknown} fn @param {RegExp} re */
const fails = (fn, re) =>
  assert.throws(fn, (/** @type {unknown} */ e) => {
    assert.ok(e instanceof XacroError, String(e));
    assert.match(e.message, re);
    return true;
  });

test("properties: values, lazy evaluation and literals", () => {
  const out = expand(`
  <xacro:property name="a" value="\${b * 2}"/>
  <xacro:property name="b" value="0.5"/>
  <xacro:property name="s" value="'x y'"/>
  <x a="\${a}" s="\${s}" t="\${1 if True else 0}"/>`);
  assert.match(out, /<x a="1.0" s="x y" t="1"\/>/);
  assert.equal(evalLiteral("3"), 3n);
  assert.equal(evalLiteral(" 2.5 "), 2.5);
  assert.equal(evalLiteral("-inf"), -Infinity);
  assert.ok(Number.isNaN(/** @type {number} */ (evalLiteral("nan"))));
  assert.equal(evalLiteral("True"), true);
  assert.equal(evalLiteral("1_0"), "1_0");
  assert.equal(evalLiteral("'q'"), "q");
  assert.equal(evalLiteral("abc"), "abc");
});

test("block properties, block parameters and insert_block", () => {
  const out = expand(`
  <xacro:property name="blk"><a/><b/></xacro:property>
  <xacro:macro name="m" params="*one **many"><wrap><xacro:insert_block name="one"/><xacro:insert_block name="many"/></wrap></xacro:macro>
  <xacro:insert_block name="blk"/>
  <xacro:m><first k="1"/><second><c/><d/></second></xacro:m>`);
  assert.match(out, /<a\/>\s*<b\/>\s*<wrap>\s*<first k="1"\/>\s*<c\/>\s*<d\/>\s*<\/wrap>/);
});

test("errors carry file:line and the macro trace", () => {
  const fs = memFs({
    "/p/main.xacro": doc(`  <xacro:include filename="lib.xacro"/>\n  <xacro:outer/>`),
    "/p/lib.xacro": doc(
      `  <xacro:macro name="outer"><xacro:inner/></xacro:macro>\n  <xacro:macro name="inner">\n    <bad v="\${undefined_thing}"/>\n  </xacro:macro>`,
    ),
  });
  try {
    expandXacro("/p/main.xacro", fs);
    assert.fail("expected an error");
  } catch (e) {
    assert.ok(e instanceof XacroError);
    assert.equal(e.file, "/p/lib.xacro");
    assert.equal(e.line, 5);
    assert.match(e.message, /^\/p\/lib\.xacro:5: NameError: name 'undefined_thing' is not defined/);
    assert.match(e.message, /when instantiating macro: inner \(\/p\/lib\.xacro:3\)/);
    assert.match(e.message, /when instantiating macro: outer \(\/p\/main\.xacro:4\)/);
  }
  fails(
    () => expand(`  <x v="\${1 +}"/>`),
    /main\.xacro:3: SyntaxError: unexpected end of expression\nwhen evaluating expression '1 \+'/,
  );
});

test("macro call errors", () => {
  const m = `<xacro:macro name="m" params="a *b"><xacro:insert_block name="b"/></xacro:macro>`;
  fails(() => expand(`${m}<xacro:m a="1" c="2"><x/></xacro:m>`), /Invalid parameter "c"/);
  fails(() => expand(`${m}<xacro:m a="1"/>`), /Not enough blocks/);
  fails(() => expand(`${m}<xacro:m a="1"><x/><y/></xacro:m>`), /Unused block "y"/);
  fails(() => expand(`${m}<xacro:m><x/></xacro:m>`), /Undefined parameters \[a\]/);
  fails(() => expand(`<xacro:nope/>`), /unknown macro name: xacro:nope/);
  fails(() => expand(`<xacro:ns.nope/>`), /unknown macro name: xacro:ns\.nope/);
  fails(() => expand(`<xacro:macro name="a.b"/>`), /must not contain '\.'/);
  fails(() => expand(`<xacro:macro name="call"/>`), /Invalid use of macro name 'call'/);
  fails(() => expand(`<xacro:call/>`), /missing attribute 'macro'/);
  fails(() => expand(`<xacro:call macro=""/>`), /missing the 'macro' attribute/);
  fails(
    () => expand(`<xacro:macro name="f" params="x:=^"><y v="\${x}"/></xacro:macro><xacro:f/>`),
    /Undefined property to forward: x/,
  );
  fails(() => expand(`<xacro:insert_block name="nothing"/>`), /Undefined block "nothing"/);
  fails(
    () => expand(`<xacro:property name="p" value="1"/><xacro:property name="*p" value="2"/>`),
    /valid python identifiers/,
  );
  fails(
    () =>
      expand(
        `<xacro:macro name="m" params="*b"><xacro:insert_block name="b"/></xacro:macro><xacro:property name="q" value="1"/><xacro:insert_block name="q"/>`,
      ),
    /Undefined block/,
  );
});

test("macro parameter parsing follows xacro's grammar", () => {
  assert.deepEqual(parseMacroArg("a b"), ["a", null, "b"]);
  assert.deepEqual(parseMacroArg("a:=1 b"), ["a", [null, "1"], "b"]);
  assert.deepEqual(parseMacroArg("a=1"), ["a", [null, "1"], ""]);
  assert.deepEqual(parseMacroArg("a:=^ b"), ["a", ["a", null], "b"]);
  assert.deepEqual(parseMacroArg("a:=^|2.5"), ["a", ["a", "2.5"], ""]);
  assert.deepEqual(parseMacroArg("a:='x y' z"), ["a", [null, "'x y'"], "z"]);
  assert.deepEqual(parseMacroArg("a:=${1 + 2} z"), ["a", [null, "${1 + 2}"], "z"]);
  assert.deepEqual(parseMacroArg("  *blk"), ["*blk", null, ""]);
});

test("dynamic and namespaced macro calls", () => {
  const out = expand(
    `<xacro:include filename="lib.xacro" ns="lib"/>
  <xacro:lib.part n="1"/>
  <xacro:property name="which" value="lib.part"/>
  <xacro:call macro="\${which}" n="2"/>
  <v s="\${lib.size}"/>`,
    {
      files: {
        "/pkg/lib.xacro": doc(
          `<xacro:property name="size" value="3"/><xacro:macro name="part" params="n"><part n="\${n}" s="\${size}"/></xacro:macro>`,
        ),
      },
    },
  );
  assert.match(out, /<part n="1" s="3"\/>\s*<part n="2" s="3"\/>\s*<v s="3"\/>/);
});

test("property scopes and removal", () => {
  fails(() => expand(`<xacro:property name="x" value="1" scope="parent"/>`), /no parent scope/);
  fails(() => expand(`<xacro:property name="x" value="1" default="2"/>`), /mutually exclusive/);
  fails(() => expand(`<xacro:property name="__x" value="1"/>`), /double underscore/);
  fails(() => expand(`<xacro:property name="if" value="1"/>`), /valid python identifiers/);
  fails(() => expand(`<xacro:property value="1"/>`), /missing attribute 'name'/);
  const out = expand(
    `<xacro:property name="g" value="1"/><xacro:property name="g" remove="true"/><xacro:property name="g" default="2"/><x g="\${g}"/>`,
  );
  assert.match(out, /<x g="2"\/>/);
  fails(
    () =>
      expand(
        `<xacro:property name="a" value="\${b}"/><xacro:property name="b" value="\${a}"/><x v="\${a}"/>`,
      ),
    /circular variable definition: a -> b -> a/,
  );
});

test("conditionals accept only boolean-like values", () => {
  assert.match(
    expand(
      `<xacro:if value="1"><a/></xacro:if><xacro:unless value="True"><b/></xacro:unless><xacro:if value=" 2 "><c/></xacro:if>`,
    ),
    /<a\/>\s*<c\/>/,
  );
  fails(
    () => expand(`<xacro:if value="maybe"><a/></xacro:if>`),
    /Xacro conditional "maybe" evaluated to "maybe", which is not a boolean expression/,
  );
  fails(() => expand(`<xacro:if value="1.0"><a/></xacro:if>`), /not a boolean expression/);
  fails(() => expand(`<xacro:if><a/></xacro:if>`), /missing attribute 'value'/);
});

test("substitution args: $(arg), $(eval), $(env), $(optenv), $(dirname), $(cwd), $(find)", () => {
  const out = expand(
    `<xacro:arg name="w" default="0.5"/><xacro:arg name="flag" default="true"/><xacro:arg name="n" default="3"/>
  <v arg="$(arg w)" eval="$(eval w * 2)" evb="$(eval flag and True)" evn="$(eval n + 1)" evs="$(eval 'a' + name)"
     env="$(env HOME_X)" opt="$(optenv NOPE default words)" opt2="$(optenv HOME_X)" dir="$(dirname)" cwd="$(cwd)"
     find="$(find pkg)/x" mixed="a$(arg w)b" dollar="$$(arg w)" pi="$(eval pi)"/>`,
    {
      args: { name: "bc" },
      env: { HOME_X: "/home/x" },
      files: { "/pkg/package.xml": "<package><name>pkg</name></package>" },
    },
  );
  assert.match(out, /arg="0.5"/);
  assert.match(out, /eval="1.0"/);
  assert.match(out, /evb="True"/);
  assert.match(out, /evn="4"/);
  assert.match(out, /evs="abc"/);
  assert.match(out, /env="\/home\/x"/);
  assert.match(out, /opt="default words"/);
  assert.match(out, /opt2="\/home\/x"/);
  assert.match(out, /dir="\/pkg"/);
  assert.match(out, /cwd="\/"/);
  assert.match(out, /find="\/pkg\/x"/);
  assert.match(out, /mixed="a0.5b"/);
  assert.match(out, /dollar="\$\(arg w\)"/);
  assert.match(out, /pi="3.141592653589793"/);
  fails(() => expand(`<v a="$(arg nope)"/>`), /Undefined substitution argument nope/);
  fails(() => expand(`<v a="$(arg)"/>`), /must specify a variable name/);
  fails(() => expand(`<v a="$(arg a b)"/>`), /may only specify one arg/);
  fails(() => expand(`<v a="$(env NOPE)"/>`), /environment variable NOPE is not set/);
  fails(() => expand(`<v a="$(env)"/>`), /only accepts one argument/);
  fails(() => expand(`<v a="$(optenv)"/>`), /must specify an environment variable/);
  fails(() => expand(`<v a="$(find nopkg)"/>`), /package 'nopkg' not found/);
  fails(() => expand(`<v a="$(find)"/>`), /exactly one argument/);
  fails(() => expand(`<v a="$(bogus x)"/>`), /Unknown substitution command \[bogus x\]/);
  fails(() => expand(`<v a="$(eval __import__)"/>`), /double underscore/);
  fails(() => expand(`<v a="$(eval nope + 1)"/>`), /name 'nope' is not defined/);
  fails(() => expand(`<v a="\${x"/>`), /invalid expression: \$\{x/);
  fails(() => expand(`<xacro:arg name="a"/>`), /missing attribute 'default'/);
});

test("$(eval) helper functions read args with automatic types", () => {
  const out = expand(
    `<xacro:arg name="x" default="2"/><v a="$(eval x * 2.5)" b="$(eval true)" c="$(eval x == 2)"/>`,
  );
  assert.match(out, /a="5.0" b="True" c="True"/);
});

test("xacro.* helpers, deprecated direct calls and unsupported load_yaml", () => {
  const fs = memFs({
    "/p/main.xacro": doc(`<xacro:arg name="a" default="7"/>
  <v arg="\${xacro.arg('a')}" tok="\${xacro.tokenize('a b,c')}" tok2="\${xacro.tokenize('a;;b', ';', False)}" abs="\${xacro.abs_filename('m.stl')}"
     dot="\${xacro.dotify(dict(k=1)).k}" dotget="\${xacro.dotify(dict(k=1)).get('k')}" msg="\${xacro.message('hello', 1)}" warn="\${xacro.warning('careful')}"
     loc="\${xacro.print_location()}" srt="\${sorted([2, 1])}" rng="\${range(2)}" dep="\${abs_filename('q')}"/>`),
  });
  const r = expandXacro("/p/main.xacro", fs);
  const out = serializeXml(r.root);
  assert.match(out, /arg="7"/);
  assert.match(out, /tok="\['a', 'b', 'c'\]"/);
  assert.match(out, /tok2="\['a', '', 'b'\]"/);
  assert.match(out, /abs="\/p\/m.stl"/);
  assert.match(out, /dot="1" dotget="1"/);
  assert.match(out, /msg=""/);
  assert.match(out, /srt="\[1, 2\]"/);
  assert.match(out, /rng="range\(0, 2\)"/);
  assert.match(out, /dep="\/p\/q"/);
  const texts = r.messages.map((m) => `${m.level}:${m.text}`);
  assert.ok(texts.includes("message:hello 1"));
  assert.ok(texts.includes("warning:careful"));
  assert.ok(texts.some((t) => /Using sorted\(\) directly is deprecated/.test(t)));
  assert.ok(texts.some((t) => /Using abs_filename\(\) directly is deprecated/.test(t)));
  fails(() => expand(`<v a="\${xacro.load_yaml('x.yaml')}"/>`), /load_yaml\(\) is not supported/);
  fails(() => expand(`<v a="\${load_yaml('x.yaml')}"/>`), /load_yaml\(\) is not supported/);
  fails(() => expand(`<v a="\${xacro.fatal('stop', 1)}"/>`), /stop 1/);
  fails(() => expand(`<v a="\${xacro.arg('zz')}"/>`), /KeyError/);
  fails(() => expand(`<v a="\${xacro.dotify(1)}"/>`), /expects a dict/);
  fails(() => expand(`<v a="\${xacro.dotify(dict(k=1)).z}"/>`), /has no attribute 'z'/);
});

test("warnings: unknown attributes, redefined globals, include children", () => {
  const fs = memFs({
    "/p/main.xacro":
      doc(`<xacro:property name="pi" value="3" bogus="1"/><xacro:include filename="i.xacro"><child/></xacro:include>
  <xacro:macro name="xacro:old"><o/></xacro:macro><xacro:old/><xacro:property name="d" default="1" scope="global"/>`),
    "/p/i.xacro": doc(""),
  });
  const r = expandXacro("/p/main.xacro", fs);
  const texts = r.messages.map((m) => m.text);
  assert.ok(texts.some((t) => /xacro:property: unknown attribute\(s\): bogus/.test(t)));
  assert.ok(texts.some((t) => /redefining global symbol: pi/.test(t)));
  assert.ok(texts.some((t) => /Child elements of a <xacro:include> tag are ignored/.test(t)));
  assert.ok(texts.some((t) => /must not contain prefix 'xacro:'/.test(t)));
  assert.ok(texts.some((t) => /default property value can only be defined on local scope/.test(t)));
  assert.deepEqual(r.files, ["/p/main.xacro", "/p/i.xacro"]);
});

test("includes: relative, $(find), glob, optional, missing and malformed files", () => {
  const files = {
    "/ws/robot/package.xml": "<package><name>robot</name></package>",
    "/ws/robot/urdf/main.xacro": doc(
      `<xacro:include filename="$(find parts)/urdf/*.xacro"/><xacro:include filename="sub/inc.xacro"/><xacro:include filename="nope.xacro" optional="true"/><r a="\${pa}" b="\${pb}" c="\${inc}"/>`,
    ),
    "/ws/robot/urdf/sub/inc.xacro": doc(`<xacro:property name="inc" value="\${pa}!"/>`),
    "/ws/parts/package.xml": "<package><name>parts</name></package>",
    "/ws/parts/urdf/b.xacro": doc(`<xacro:property name="pb" value="\${pa}B"/>`),
    "/ws/parts/urdf/a.xacro": doc(`<xacro:property name="pa" value="A"/>`),
  };
  const out = serializeXml(expandXacro("/ws/robot/urdf/main.xacro", memFs(files)).root);
  assert.match(out, /<r a="A" b="AB" c="A!"\/>/);
  const r = expandXacro("/ws/robot/urdf/main.xacro", memFs(files));
  assert.ok(r.messages.length === 0);
  fails(
    () => expand(`<xacro:include filename="missing.xacro"/>`),
    /No such file or directory: \/pkg\/missing\.xacro/,
  );
  fails(
    () =>
      expand(`<xacro:include filename="bad.xacro"/>`, {
        files: { "/pkg/bad.xacro": "<robot><a></robot>" },
      }),
    /XML parsing error: .*bad\.xacro:1:.*does not match/,
  );
  const warned = expandXacro(
    "/m.xacro",
    memFs({ "/m.xacro": doc(`<xacro:include filename="none/*.xacro"/>`) }),
  );
  assert.ok(warned.messages.some((m) => /matched no files/.test(m.text)));
});

test("install-layout fallback for $(find) into a package's share folder", () => {
  const fs = memFs({
    "/demo/package.xml": "<package><name>demo</name></package>",
    "/demo/description/urdf/robot.xacro": doc(
      `<xacro:include filename="$(find demo)/parts/p.xacro"/><xacro:p/>`,
    ),
    "/demo/description/parts/p.xacro": doc(`<xacro:macro name="p"><p/></xacro:macro>`),
  });
  const out = serializeXml(expandXacro("/demo/description/urdf/robot.xacro", fs).root);
  assert.match(out, /<p\/>/);
  assert.equal(fs.notes.length, 1);
  assert.match(fs.notes[0].message, /using \/demo\/description\/parts\/p\.xacro/);
});

test("xacro:element, xacro:attribute, targetNamespace, comments and text", () => {
  const out = expand(`<xacro:property name="n" value="2"/>
  <!-- documents the property, removed with it -->
  <xacro:property name="m" value="1"/>
  <!-- kept -->
  <xacro:element xacro:name="tag\${n}" a="1"><xacro:attribute name="b" value="\${n * 2}"/><![CDATA[\${raw}]]></xacro:element>
  <!-- xacro:eval-comments -->
  <!-- n=\${n} -->
  <t>n is \${n}</t>`);
  assert.doesNotMatch(out, /documents the property/);
  assert.match(out, /<!-- kept -->/);
  assert.match(out, /<tag2 a="1" b="4">\n<!\[CDATA\[\$\{raw\}\]\]>/);
  assert.match(out, /<!-- n=2 -->/);
  assert.doesNotMatch(out, /eval-comments/);
  assert.match(out, /<t>n is 2<\/t>/);
  fails(() => expand(`<xacro:element a="1"/>`), /missing attribute 'xacro:name'/);
  fails(() => expand(`<xacro:element xacro:name=""/>`), /empty name/);
  fails(() => expand(`<xacro:attribute name="" value="1"/>`), /empty name/);
  const ns = expandXacro(
    "/a.xacro",
    memFs({
      "/a.xacro": `<robot xmlns:xacro="http://www.ros.org/wiki/xacro" xacro:targetNamespace="urn:x" xmlns:extra="urn:e"/>`,
    }),
  );
  assert.deepEqual(ns.root.attrs, { "xmlns:extra": "urn:e", xmlns: "urn:x" });
});

test("serialisation follows xacro's pretty printer, conversion keeps lines and files", () => {
  const fs = memFs({
    "/a.xacro": `<?xml version="1.0"?>\n<!-- head -->\n<robot name="r" b="2" a="&quot;1&quot;">\n  <link name="l">text &amp; &lt;</link>\n  <x>  </x>\n  <y>a<z/>b</y>\n</robot>`,
  });
  const r = expandXacro("/a.xacro", fs);
  assert.equal(
    serializeXml(r.root),
    `<?xml version="1.0" ?>\n<!-- head -->\n<robot a="&quot;1&quot;" b="2" name="r">\n  <link name="l">text &amp; &lt;</link>\n  <x>  </x>\n  <y>\n    a\n    <z/>\n    b\n  </y>\n</robot>\n`,
  );
  const root = toXmlElement(r.root);
  assert.equal(root.children[0].line, 4);
  assert.equal(root.children[0].text, "text & <");
  assert.equal(root.children[0].source, "/a.xacro");
});

test("the document reader: syntax errors with positions, normalisation, entities", () => {
  assert.throws(
    () => parseDocument("<a>\n  <b>\n</a>", "f.xml"),
    (/** @type {unknown} */ e) =>
      e instanceof XmlSyntaxError && e.line === 3 && /does not match <b>/.test(e.message),
  );
  for (const [src, re] of /** @type {[string, RegExp][]} */ ([
    ["", /no root element/],
    ["<a></a><b/>", /content after the root element/],
    ["<a b=1/>", /must be quoted/],
    ["<a b/>", /has no value/],
    ['<a b="1" b="2"/>', /duplicate attribute/],
    ['<a b="<"/>', /'<' inside the value/],
    ['<a b="x/>', /unterminated value/],
    ["<a>&bogus;</a>", /unknown entity/],
    ["<a>&#xFFFFFFF;</a>", /bad character reference/],
    ["<a>a & b</a>", /bare '&'/],
    ["<a><!-- x </a>", /unterminated comment/],
    ["<a><![CDATA[x</a>", /unterminated CDATA/],
    ["<a>", /is not closed/],
    ['<a x="1"y="2"/>', /expected whitespace/],
    ["<!DOCTYPE a [<!ENTITY x 'y'>]><a/>", /internal subset/],
    ["<1a/>", /expected a name/],
    ["<a></a >x", /content after/],
  ]))
    assert.throws(() => parseDocument(src), re, src);
  const el = parseDocument(
    '﻿<?xml version="1.0"?>\r\n<!DOCTYPE robot>\r\n<a v="x\ty\nz" w="&#65;&#x42;&lt;"><?pi skip?>t&gt;</a>\r\n<!-- trailing -->',
  );
  assert.deepEqual(el.attrs, { v: "x y z", w: "AB<" });
  assert.deepEqual(el.children, [{ type: "text", value: "t>" }]);
  const copy = cloneNode(el);
  copy.attrs.v = "changed";
  assert.equal(el.attrs.v, "x y z");
});

test("looksLikeXacro tells plain URDF from xacro", () => {
  assert.equal(looksLikeXacro('<robot name="r"><link name="a"/></robot>'), false);
  assert.equal(looksLikeXacro('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>'), true);
  assert.equal(looksLikeXacro('<robot><link name="${n}"/></robot>'), true);
  assert.equal(looksLikeXacro("<robot><xacro:if value='1'/></robot>"), true);
});

test("a missing entry file and a malformed entry file are reported", () => {
  fails(() => expandXacro("/nope.xacro", memFs({})), /No such file or directory: \/nope\.xacro/);
  fails(
    () => expandXacro("/bad.xacro", memFs({ "/bad.xacro": "<robot>" })),
    /bad\.xacro:1: XML parsing error/,
  );
});
