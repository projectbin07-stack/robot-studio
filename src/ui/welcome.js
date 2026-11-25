// First-run panel over the 3D view: the three things the studio does
// (open or build a robot, simulate it, connect to ROS 2), each one click
// away, and a guided example that walks through them with the bundled
// mecanum robot. Shown on a first visit without a link's state; "Start here"
// in the header brings it back. Every step does what it says in the page
// itself (no screenshots, no claims about robots it has not seen).

const DISMISSED_KEY = "robot-studio:welcome-dismissed";

/**
 * @typedef {object} TourStep
 * @property {string} title
 * @property {string} text
 * @property {() => void | Promise<void>} run what the step does in the page
 */

/** @param {string} id */
const $ = (id) => /** @type {HTMLElement} */ (document.getElementById(id));

/** @param {string} id */
const click = (id) => /** @type {HTMLElement} */ ($(id)).click();

/**
 * @param {string} id
 * @param {string} value
 */
function choose(id, value) {
  const el = /** @type {HTMLSelectElement} */ ($(id));
  el.value = value;
  el.dispatchEvent(new Event("change"));
}

/** @param {string} id */
function openPanel(id) {
  const d = /** @type {HTMLDetailsElement} */ ($(id));
  d.open = true;
  d.scrollIntoView({ block: "nearest" });
}

/**
 * Wait until the page has finished loading or rebuilding the robot.
 * @param {number} [ms]
 */
async function settle(ms = 15000) {
  const t0 = Date.now();
  await new Promise((r) => setTimeout(r, 50));
  while (document.body.dataset.state !== "ready" && Date.now() - t0 < ms)
    await new Promise((r) => setTimeout(r, 100));
}

/** @type {TourStep[]} */
const STEPS = [
  {
    title: "1 of 4 · Open a robot",
    text: "The bundled example is open: a four-wheel mecanum base written as xacro (four files and two meshes), expanded in this page. The Kinematic tree lists its links and joints (arrow keys move, Enter selects), Findings lists urdf-inspector's checks, and Joints has a slider for every movable joint. Your own model: drop a URDF, a xacro file, a package folder or a .zip on the Model panel.",
    async run() {
      choose("example", "mecanum");
      await settle();
      openPanel("p-tree");
      const first = /** @type {HTMLElement | null} */ (
        document.querySelector("#tree [role=treeitem]")
      );
      first?.focus({ preventScroll: true });
    },
  },
  {
    title: "2 of 4 · Build one",
    text: "A robot from the mecanum template. Change a value in the Build panel and the robot is regenerated (Ctrl+Z undoes it); the Properties panel edits any link or joint. Export writes URDF, xacro, a mecanum-kinematics config or a ROS 2 description package.",
    async run() {
      openPanel("p-build");
      choose("template", "mecanum");
      click("btn-create");
      await settle();
    },
  },
  {
    title: "3 of 4 · Simulate it",
    text: "The same robot drives the demo route in a simulated office: LiDAR rays, the map it builds, and three estimates (wheel odometry, odometry with scan matching, MCL) against the simulated truth. Drive by hand with W/A/S/D and Q/E. Everything in this mode is model output and labelled simulated.",
    async run() {
      openPanel("p-sim");
      if ($("sim-controls").hidden) click("sim-open");
      await new Promise((r) => setTimeout(r, 300));
      choose("sim-world", "office");
      choose("sim-route", "demo");
    },
  },
  {
    title: "4 of 4 · Connect to ROS 2",
    text: "For a real robot, run rosbridge_server on it and enter its address here; the page never connects by itself, and driving needs a separate opt-in. From this https page the browser asks to allow access to your local network; if it cannot, start the studio on your computer with npx github:projectbin07-stack/robot-studio studio. Without a robot: download the simulation's recording and replay it here. No real robot has been connected to this studio yet; docs/ros2.md has the steps.",
    async run() {
      const close = /** @type {HTMLButtonElement | null} */ (document.getElementById("sim-close"));
      if (close && !$("sim-controls").hidden) close.click();
      openPanel("p-ros");
      $("ros-url").focus({ preventScroll: true });
    },
  },
];

/**
 * @param {{ firstVisit: boolean }} opts firstVisit: no state in the link
 */
export function initWelcome(opts) {
  const panel = $("welcome");
  const tour = $("welcome-tour");
  const cards = $("welcome-cards");
  let at = -1;

  const dismissed = () => {
    try {
      return localStorage.getItem(DISMISSED_KEY) === "1";
    } catch {
      return false;
    }
  };
  const show = () => {
    panel.hidden = false;
    cards.hidden = false;
    tour.hidden = true;
    panel.classList.remove("touring");
    at = -1;
    document.body.dataset.welcome = "open";
  };
  const hide = () => {
    panel.hidden = true;
    document.body.dataset.welcome = "closed";
    try {
      localStorage.setItem(DISMISSED_KEY, "1");
    } catch {
      // storage blocked: the panel just comes back next time
    }
  };
  /** @param {number} k */
  const go = async (k) => {
    at = Math.max(0, Math.min(STEPS.length - 1, k));
    cards.hidden = true;
    tour.hidden = false;
    panel.classList.add("touring");
    const s = STEPS[at];
    $("tour-title").textContent = s.title;
    $("tour-text").textContent = s.text;
    /** @type {HTMLButtonElement} */ ($("tour-back")).disabled = at === 0;
    $("tour-next").textContent = at === STEPS.length - 1 ? "Finish" : "Next";
    document.body.dataset.tourStep = String(at + 1);
    $("tour-status").textContent = "working…";
    try {
      await s.run();
      $("tour-status").textContent = "";
    } catch (e) {
      $("tour-status").textContent =
        `This step did not complete: ${e instanceof Error ? e.message : String(e)}`;
    }
    $("tour-next").focus({ preventScroll: true });
  };

  $("welcome-close").addEventListener("click", hide);
  $("tour-end").addEventListener("click", hide);
  $("btn-welcome").addEventListener("click", () => {
    show();
    $("welcome-start").focus();
  });
  $("welcome-start").addEventListener("click", () => void go(0));
  $("tour-back").addEventListener("click", () => void go(at - 1));
  $("tour-next").addEventListener("click", () => {
    if (at >= STEPS.length - 1) hide();
    else void go(at + 1);
  });
  $("welcome-open").addEventListener("click", () => {
    hide();
    openPanel("p-model");
    /** @type {HTMLInputElement} */ ($("file")).click();
  });
  $("welcome-build").addEventListener("click", () => {
    hide();
    openPanel("p-build");
    $("template").focus();
  });
  $("welcome-sim").addEventListener("click", async () => {
    hide();
    await STEPS[2].run();
  });
  $("welcome-ros").addEventListener("click", () => {
    hide();
    openPanel("p-ros");
    $("ros-url").focus();
  });
  panel.addEventListener("keydown", (e) => {
    if (e.key === "Escape") hide();
  });
  if (opts.firstVisit && !dismissed()) show();
  else document.body.dataset.welcome = "closed";
}
