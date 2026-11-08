(() => {
  const path = window.location.pathname.replace(/\/+$/, "");
  if (!path.endsWith("/physical-ai-learner-lab")) return;
  if (window.matchMedia("(prefers-reduced-motion: reduce)").matches) return;

  const key = "physical-ai-lab-intro-seen";
  try {
    if (sessionStorage.getItem(key)) return;
  } catch (_) {
    // Continue without session storage when browser privacy settings block it.
  }

  const overlay = document.createElement("div");
  overlay.className = "lab-intro";
  overlay.setAttribute("role", "dialog");
  overlay.setAttribute("aria-label", "Physical AI Learner Lab introduction");

  const row = document.createElement("div");
  row.className = "lab-intro__row";
  const mark = document.createElement("img");
  mark.className = "lab-intro__mark";
  mark.src = new URL("assets/icosa-shell.svg", document.baseURI).href;
  mark.alt = "";
  const copy = document.createElement("div");
  const eyebrow = document.createElement("p");
  eyebrow.className = "lab-intro__eyebrow";
  eyebrow.textContent = "SUPASH BHAT · ROBOTICS";
  const title = document.createElement("h1");
  title.className = "lab-intro__title";
  title.textContent = "Physical AI Learner Lab";
  const line = document.createElement("p");
  line.className = "lab-intro__line";
  line.textContent = "A small robot, from model to motion.";
  const skip = document.createElement("button");
  skip.className = "lab-intro__skip";
  skip.type = "button";
  skip.textContent = "Skip intro";

  copy.append(eyebrow, title, line);
  row.append(mark, copy);
  overlay.append(row, skip);
  document.body.append(overlay);

  let finished = false;
  const finish = () => {
    if (finished) return;
    finished = true;
    try { sessionStorage.setItem(key, "1"); } catch (_) {}
    overlay.classList.remove("is-visible");
    window.setTimeout(() => overlay.remove(), 400);
  };
  skip.addEventListener("click", finish, { once: true });
  window.requestAnimationFrame(() => overlay.classList.add("is-visible"));
  window.setTimeout(finish, 1450);
})();
