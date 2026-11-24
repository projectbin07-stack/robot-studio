// The safe expression evaluator: Python semantics for what xacro files
// use, and rejection of everything outside the grammar.

import { test } from "node:test";
import assert from "node:assert/strict";
import { py } from "./helpers.js";
import { parseExpression, tokenize } from "../src/core/xacro/pyexpr.js";
import {
  PyDict,
  PyError,
  PyTuple,
  binary,
  fixedHalfEven,
  floatRepr,
  formatFloat,
  pyCompare,
  pyEq,
  pyRepr,
  strRepr,
  truthy,
  typeName,
  unary,
} from "../src/core/xacro/pyvalue.js";

test("float repr matches Python's shortest round-trip format", () => {
  const cases = /** @type {[number, string][]} */ ([
    [0.1, "0.1"],
    [2, "2.0"],
    [1 / 3, "0.3333333333333333"],
    [0.1 + 0.2, "0.30000000000000004"],
    [1e16, "1e+16"],
    [1e15, "1000000000000000.0"],
    [1e-5, "1e-05"],
    [1.5e-7, "1.5e-07"],
    [0.0001, "0.0001"],
    [-0.0, "-0.0"],
    [0, "0.0"],
    [123456789.123, "123456789.123"],
    [1.2345678901234568e17, "1.2345678901234568e+17"],
    [-2.5e-300, "-2.5e-300"],
    [Infinity, "inf"],
    [-Infinity, "-inf"],
    [NaN, "nan"],
  ]);
  for (const [x, s] of cases) assert.equal(floatRepr(x), s, String(x));
});

test("numbers: int/float distinction and Python division, modulo and power", () => {
  assert.equal(py("6/3"), "2.0");
  assert.equal(py("2*3"), "6");
  assert.equal(py("7//2"), "3");
  assert.equal(py("-7//2"), "-4");
  assert.equal(py("-7%3"), "2");
  assert.equal(py("7%-3"), "-2");
  assert.equal(py("7.5%-2"), "-0.5");
  assert.equal(py("-7.5//2"), "-4.0");
  assert.equal(py("2**10"), "1024");
  assert.equal(py("2**-1"), "0.5");
  assert.equal(py("-2**2"), "-4");
  assert.equal(py("2**3**2"), "512");
  assert.equal(py("2**100"), "1267650600228229401496703205376");
  assert.equal(py("True + 1"), "2");
  assert.equal(py("0x1f + 0o7 + 0b11"), "41");
  assert.equal(py("1_000 * 2"), "2000");
  assert.equal(py("5."), "5.0");
  assert.equal(py(".5"), "0.5");
  assert.equal(py("1e3"), "1000.0");
  assert.equal(py("~5"), "-6");
  assert.equal(py("+3"), "3");
  assert.equal(py("6 | 1, 6 & 3, 6 ^ 3, 1 << 4, 256 >> 2"), "(7, 2, 5, 16, 64)");
  assert.throws(() => py("1/0"), /ZeroDivisionError/);
  assert.throws(() => py("1//0"), /ZeroDivisionError/);
  assert.throws(() => py("1.0%0"), /ZeroDivisionError/);
  assert.throws(() => py("0**-1"), /ZeroDivisionError/);
  assert.throws(() => py("0.0**-1"), /ZeroDivisionError/);
  assert.throws(() => py("(-8)**0.5"), /ValueError/);
  assert.throws(() => py("10.0**400"), /OverflowError/);
  assert.throws(() => py("1 << -1"), /ValueError/);
  assert.throws(() => py("1 >> -1"), /ValueError/);
  assert.throws(() => py("~1.5"), /TypeError/);
  assert.throws(() => py("-'a'"), /TypeError/);
  assert.throws(() => py("'a' - 'b'"), /unsupported operand type\(s\) for -: 'str' and 'str'/);
  assert.throws(() => py("1.5 << 1"), /unsupported operand/);
});

test("strings, containers and comparisons behave like Python", () => {
  assert.equal(py("'a' + 'b'"), "ab");
  assert.equal(py("'ab' * 2"), "abab");
  assert.equal(py("2 * [1]"), "[1, 1]");
  assert.equal(py("(1,) * 2"), "(1, 1)");
  assert.equal(py("'robot'[1:3]"), "ob");
  assert.equal(py("'robot'[::-1]"), "tobor");
  assert.equal(py("[1, 2, 3, 4][1:]"), "[2, 3, 4]");
  assert.equal(py("(1, 2, 3)[-2:]"), "(2, 3)");
  assert.equal(py("'%.3f' % 1.23456"), "1.235");
  assert.equal(py("'%s-%d-%i-%r-%%' % ('a', 2.9, 3, 'q')"), "a-2-3-'q'-%");
  assert.equal(py("'%e' % 12345.678"), "1.234568e+04");
  assert.equal(py("'%g|%g|%g' % (0.0001, 1234567.0, 0.5)"), "0.0001|1.23457e+06|0.5");
  assert.throws(() => py("'%s %s' % ('a',)"), /not enough arguments/);
  assert.throws(() => py("'%s' % ('a', 'b')"), /not all arguments/);
  assert.throws(() => py("'%d' % 'x'"), /real number is required/);
  assert.equal(py("'{}_{}'.format('a', 2)"), "a_2");
  assert.equal(py("'{0}{1}{0}'.format('x', 'y')"), "xyx");
  assert.equal(py("'{n}'.format(n=3)"), "3");
  assert.equal(
    py("'{:.2f}|{:d}|{:g}|{:.1%}|{{}}'.format(2.675, 7, 0.5, 0.25)"),
    "2.67|7|0.5|25.0%|{}",
  );
  assert.throws(() => py("'{:x}'.format(1)"), /not supported/);
  assert.throws(() => py("'{:d}'.format(1.5)"), /Unknown format code/);
  assert.throws(() => py("'{}{}'.format(1)"), /out of range/);
  assert.equal(
    py("' a b '.strip() + '|' + 'xxa'.lstrip('x') + '|' + 'axx'.rstrip('x')"),
    "a b|a|a",
  );
  assert.equal(py("'a,b,,c'.split(',')"), "['a', 'b', '', 'c']");
  assert.equal(py("'a b c'.split(None, 1)"), "['a', 'b c']");
  assert.equal(py("'a,b,c'.split(',', 1)"), "['a', 'b,c']");
  assert.throws(() => py("'a'.split('')"), /empty separator/);
  assert.equal(py("'-'.join(['a', 'b'])"), "a-b");
  assert.equal(py("'aXbX'.replace('X', '_')"), "a_b_");
  assert.equal(py("'abc'.upper() + 'ABC'.lower()"), "ABCabc");
  assert.equal(py("('ab'.find('b'), 'abab'.count('ab'), 'ab'.endswith('b'))"), "(1, 2, True)");
  assert.throws(() => py("'a'.nosuch"), /has no attribute 'nosuch'/);
  assert.throws(() => py("'a'.startswith(1)"), /must be str/);
  assert.equal(py("[1, 'a', 2.0, None, True]"), "[1, 'a', 2.0, None, True]");
  assert.equal(py('["it\'s"]'), `["it's"]`);
  assert.equal(py("('a\\nb',)"), "('a\\nb',)");
  assert.equal(py("dict(a=1, b='x')"), "{'a': 1, 'b': 'x'}");
  assert.equal(py("{'k': 1, 2: (3,)}"), "{'k': 1, 2: (3,)}");
  assert.equal(py("dict([('a', 1)])['a']"), "1");
  assert.equal(py("dict(dict(a=1), b=2)"), "{'a': 1, 'b': 2}");
  assert.equal(py("dict(a=1).get('a'), dict(a=1).get('z')"), "(1, None)");
  assert.equal(py("list(dict(a=1, b=2).keys()) + list(dict(a=1).values())"), "['a', 'b', 1]");
  assert.equal(py("dict(a=1).items()"), "[('a', 1)]");
  assert.equal(py("[1, 2].index(2), [1, 1].count(1)"), "(1, 2)");
  assert.throws(() => py("[1].index(5)"), /is not in list/);
  assert.throws(() => py("dict(a=1)['b']"), /KeyError/);
  assert.throws(() => py("[1][3]"), /IndexError/);
  assert.throws(() => py("[1]['a']"), /indices must be integers/);
  assert.throws(() => py("5[0]"), /not subscriptable/);
  assert.throws(() => py("5[0:1]"), /not subscriptable/);
  assert.throws(() => py("[1][::0]"), /step cannot be zero/);
  assert.throws(() => py("dict([1])"), /TypeError|ValueError/);
  assert.throws(() => py("dict([(1, 2, 3)])"), /wrong length/);
  assert.throws(() => py("dict(1, 2)"), /at most 1/);
  assert.throws(() => py("{[1]: 2}"), /unhashable/);
  assert.equal(
    py("1 < 2 < 3, 1 < 3 < 2, 1 == 1.0, 'a' < 'b', [1, 2] < [1, 3], (1,) < (1, 2)"),
    "(True, False, True, True, True, True)",
  );
  assert.equal(
    py("2 >= 2, 3 > 2, 2 <= 1, 1 != 2, [1] == [1], (1, 2) == (1, 2), dict(a=1) == dict(a=1)"),
    "(True, True, False, True, True, True, True)",
  );
  assert.equal(
    py("'b' in 'abc', 2 in [1, 2], 'a' in dict(a=1), 3 not in (1, 2), None is None, 1 is not None"),
    "(True, True, True, True, True, True)",
  );
  assert.equal(py("float('nan') < 1, float('nan') > 1"), "(False, False)");
  assert.throws(() => py("1 < 'a'"), /'<' not supported between instances of 'int' and 'str'/);
  assert.throws(() => py("1 in 'a'"), /requires string/);
  assert.throws(() => py("1 in 5"), /not iterable/);
});

test("logic returns operands, conditional expressions and truthiness", () => {
  assert.equal(py("0 and 5"), "0");
  assert.equal(py("0 or 5"), "5");
  assert.equal(py("3 and 4"), "4");
  assert.equal(py("'' or 'x'"), "x");
  assert.equal(py("not []"), "True");
  assert.equal(py("1 if 0 else 2 if 1 else 3"), "2");
  assert.equal(py("x if c else y", { x: 1n, y: 2n, c: false }), "2");
  for (const [v, t] of /** @type {[any, boolean][]} */ ([
    [0n, false],
    [0, false],
    ["", false],
    [[], false],
    [new PyTuple([]), false],
    [new PyDict(), false],
    [null, false],
    [1n, true],
    ["0", true],
    [[0n], true],
  ]))
    assert.equal(truthy(v), t);
});

test("builtins and math functions exposed by xacro", () => {
  assert.equal(
    py(
      "int(2.7), int(-2.7), int('42'), int(' -7 '), int('ff', 16), int('0b101', 2), int(True), int()",
    ),
    "(2, -2, 42, -7, 255, 5, 1, 0)",
  );
  assert.throws(() => py("int('1.5')"), /invalid literal/);
  assert.throws(() => py("int(float('inf'))"), /OverflowError/);
  assert.throws(() => py("int(float('nan'))"), /ValueError/);
  assert.throws(() => py("int(1.5, 10)"), /explicit base/);
  assert.equal(
    py("float('3'), float(2), float(' 1e-3 '), float('-inf'), float()"),
    "(3.0, 2.0, 0.001, -inf, 0.0)",
  );
  assert.throws(() => py("float('abc')"), /could not convert/);
  assert.equal(py("str(1.5) + str(7) + str() + str(None)"), "1.57None");
  assert.equal(py("bool('False'), bool(0), bool()"), "(True, False, False)");
  assert.equal(
    py("round(2.5), round(3.5), round(-2.5), round(0.5), round(1.4999)"),
    "(2, 4, -2, 0, 1)",
  );
  assert.equal(
    py("round(2.675, 2), round(0.125, 2), round(0.375, 2), round(1.0, 0)"),
    "(2.67, 0.12, 0.38, 1.0)",
  );
  assert.equal(
    py("round(7, 2), round(1250, -2), round(1350, -2), round(-1250, -2), round(1234.5, -2)"),
    "(7, 1200, 1400, -1200, 1200.0)",
  );
  assert.equal(py("round(True)"), "1");
  assert.throws(() => py("round('a')"), /__round__/);
  assert.equal(py("max(1, 2.5), min([3, 1, 2]), max('ab'), min(2, 1, 3)"), "(2.5, 1, 'b', 1)");
  assert.throws(() => py("max([])"), /empty/);
  assert.equal(py("len('héllo'), len([1, 2]), len(dict(a=1)), len((1,))"), "(5, 2, 1, 1)");
  assert.equal(py("list('ab'), list(), list((1, 2))"), "(['a', 'b'], [], [1, 2])");
  assert.equal(py("list(map(float, ['1', '2.5']))"), "[1.0, 2.5]");
  assert.equal(py("map(max, [1, 5], [4, 2])"), "[4, 5]");
  assert.throws(() => py("map(float)"), /at least two/);
  assert.equal(py("python.abs(-2), python.abs(-2.5), python.abs(True)"), "(2, 2.5, 1)");
  assert.throws(() => py("python.abs('a')"), /bad operand/);
  assert.equal(py("python.sum([1, 2.5]), python.sum([[1], [2]], [])"), "(3.5, [1, 2])");
  assert.equal(
    py("python.all([1, 0]), python.any([0, 1]), python.sorted([3, 1, 2])"),
    "(False, True, [1, 2, 3])",
  );
  assert.equal(
    py(
      "python.range(3), python.range(1, 6, 2), list(python.range(3, 0, -1)), len(python.range(4))",
    ),
    "(range(0, 3), range(1, 6, 2), [3, 2, 1], 4)",
  );
  assert.throws(() => py("python.range()"), /expected 1 to 3/);
  assert.throws(() => py("python.range(1, 2, 0)"), /must not be zero/);
  assert.equal(py("list(python.zip([1, 2], 'ab')), python.zip()"), "([(1, 'a'), (2, 'b')], [])");
  assert.equal(
    py("python.enumerate('ab', 1), python.reversed([1, 2]), python.tuple([1]), python.tuple()"),
    "([(1, 'a'), (2, 'b')], [2, 1], (1,), ())",
  );
  assert.equal(
    py("python.ord('a'), python.repr('a'), python.divmod(7, -2), python.divmod(7.5, 2)"),
    "(97, \"'a'\", (-4, -1), (3.0, 1.5))",
  );
  assert.throws(() => py("python.ord('ab')"), /character/);
  assert.equal(
    py("pi, math.pi, e, tau"),
    "(3.141592653589793, 3.141592653589793, 2.718281828459045, 6.283185307179586)",
  );
  assert.equal(
    py("radians(90), degrees(pi), math.radians(180)"),
    "(1.5707963267948966, 180.0, 3.141592653589793)",
  );
  assert.equal(
    py("sqrt(2), floor(2.5), ceil(-2.5), trunc(-2.7), floor(3)"),
    "(1.4142135623730951, 2, -2, -2, 3)",
  );
  assert.equal(
    py("atan2(1, 1), hypot(3, 4), pow(2, 3), fabs(-3), fmod(-7, 3), copysign(3, -0.0)"),
    "(0.7853981633974483, 5.0, 8.0, 3.0, -1.0, -3.0)",
  );
  assert.equal(
    py("log(e), log10(1000), log(8, 2), log2(8), exp(0), cos(0), asin(1)"),
    "(1.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.5707963267948966)",
  );
  assert.equal(
    py("isclose(0.1 + 0.2, 0.3), isclose(1, 1.1, rel_tol=0.2), isclose(1, 1.1, abs_tol=0.01)"),
    "(True, True, False)",
  );
  assert.equal(
    py("factorial(5), gcd(12, 18), gcd(-4, 6), isnan(nan), isinf(inf), isfinite(1.0)"),
    "(120, 6, 2, True, True, True)",
  );
  assert.throws(() => py("sqrt(-1)"), /math domain error/);
  assert.throws(() => py("log(0)"), /math domain error/);
  assert.throws(() => py("pow(-8, 0.5)"), /math domain error/);
  assert.throws(() => py("fmod(1, 0)"), /math domain error/);
  assert.throws(() => py("factorial(-1)"), /negative/);
  assert.throws(() => py("sqrt('a')"), /must be a real number/);
  assert.throws(() => py("factorial(1.5)"), /must be an integer/);
  assert.throws(() => py("sin(1, 2)"), /takes 1 argument/);
  assert.throws(() => py("sin(x=1)"), /no keyword arguments/);
  assert.throws(() => py("math.nosuch"), /has no attribute/);
  assert.throws(() => py("nosuch"), /NameError: name 'nosuch' is not defined/);
  assert.throws(() => py("1()"), /'int' object is not callable/);
});

test("anything outside the grammar is rejected, never executed", () => {
  const rejected = [
    ["(lambda q: q)(1)", /lambda expressions are not supported/],
    ["[x for x in [1]]", /comprehensions/],
    ["(x for x in [1])", /comprehensions/],
    ["{k: 1 for k in 'a'}", /comprehensions/],
    ["(y := 1)", /assignment expressions/],
    ["__import__('os')", /invalid name/],
    ["().__class__", /invalid name/],
    ["'a'._private", /private attribute/],
    ["f'{1}'", /f-strings/],
    ["b'x'", /bytes/],
    ["1j", /complex/],
    ["...", /'\.\.\.'/],
    ["'''x'''", /triple-quoted/],
    ["import os", /'import' is not supported/],
    ["yield 1", /yield/],
    ["await x", /await/],
    ["x = 1", /assignment/],
    ["a; b", /statements/],
    ["a @ b", /@ operator/],
    ["f(*a)", /starred/],
    ["f(**a)", /starred/],
    ["{**a}", /dict unpacking/],
    ["{1, 2}", /set literals/],
    ["1 # comment", /comments/],
    ["1 + \\\n 2", /line continuation/],
    ["'unterminated", /unterminated string/],
    ["007", /leading zeros/],
    ["1abc", /invalid number/],
    ["1 +", /unexpected end/],
    ["", /empty expression/],
    ["f(a=1, 2)", /positional argument follows keyword/],
    ["a[]", /empty subscript/],
    ["a[1:2:3:4]", /unexpected/],
    ["a.1", /unexpected/],
    ["1 if 2", /expected 'else'/],
    ["$x", /unexpected character/],
    ["rb'x'", /bytes/],
    ["xu'a'", /unexpected/],
    ["'\\N{DASH}'", /\\N/],
    ["def", /'def' is not supported/],
    ["(1, 2", /unexpected end/],
  ];
  for (const [src, re] of /** @type {[string, RegExp][]} */ (rejected)) {
    assert.throws(
      () => py(src),
      (/** @type {unknown} */ e) => e instanceof PyError && re.test(e.message),
      src,
    );
  }
});

test("tokenizer and parser details", () => {
  assert.deepEqual(
    tokenize("a.b(1, 'x')").map((t) => t.v),
    ["a", ".", "b", "(", "1", ",", "'x'", ")", ""],
  );
  assert.equal(py("'a' 'b' \"c\""), "abc");
  assert.equal(py("r'a\\nb'"), "a\\nb");
  assert.equal(py("u'x' + R'y'"), "xy");
  assert.equal(py("'\\x41\\u0042\\101\\t|\\q'"), "ABA\t|\\q");
  assert.equal(py("()"), "()");
  assert.equal(py("[1, 2,]"), "[1, 2]");
  assert.equal(py("1, 2,"), "(1, 2)");
  assert.equal(py("(1, 2,)"), "(1, 2)");
  assert.equal(py("{}"), "{}");
  assert.equal(py("[1, 2][-1:][0]"), "2");
  assert.equal(py("not 1 == 2"), "True");
  assert.equal(py("1 + 2 * 3 - 4 / 2"), "5.0");
  assert.equal(parseExpression("a if b else c").k, "if");
});

test("value helpers", () => {
  assert.equal(strRepr("it's"), `"it's"`);
  assert.equal(strRepr(`a'b"c`), `'a\\'b"c'`);
  assert.equal(strRepr("\u0001\r\\"), "'\\x01\\r\\\\'");
  assert.equal(pyRepr(new PyTuple(["a"])), "('a',)");
  assert.equal(typeName(new PyDict()), "dict");
  assert.equal(pyEq(1n, 1.0), true);
  assert.equal(pyEq(true, 1n), true);
  assert.equal(pyEq("1", 1n), false);
  assert.equal(pyEq(null, null), true);
  assert.equal(pyEq([1n], new PyTuple([1n])), false);
  assert.equal(pyCompare(2n, 10n, "<"), -1);
  assert.equal(pyCompare(true, 0.5, "<"), 1);
  assert.equal(fixedHalfEven(0.125, 2), "0.12");
  assert.equal(fixedHalfEven(-0.001, 2), "-0.00");
  assert.equal(fixedHalfEven(9.995, 2), "9.99");
  assert.equal(fixedHalfEven(9.9951, 2), "10.00");
  assert.equal(fixedHalfEven(Infinity, 2), "inf");
  assert.equal(formatFloat(0, 3, "g"), "0");
  assert.equal(binary("+", new PyTuple([1n]), new PyTuple([2n])) instanceof PyTuple, true);
  assert.equal(unary("-", true), -1n);
  assert.equal(unary("-", 1.5), -1.5);
});
