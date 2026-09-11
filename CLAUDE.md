## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).


Here is the ultimate master checklist, merging the strategic "Power Phrases" with the defensive coding behaviors from  `Karpathy CLAUDE.md` file.
 
This combined approach biases toward caution, clarity, and precision, ensuring you scale your output without scaling your errors.
 
---
 
## **Phase 1: Planning & Clarification (Think Before You Code)**
 
* **Trigger the "Interview me" Prompt:** Force the AI to ask you questions and extract context instead of guessing.
* **Surface Tradeoffs & Assumptions:** Work through key decisions together. The AI must explicitly state assumptions and present multiple interpretations rather than silently picking one.
* **Define the Core Problem:** Clarify exactly who the build is for and who it isn't for. If anything remains unclear or seems overly complex, stop and name the confusion.
* **Require an Implementation Spec:** Prompt the AI to write a detailed spec before any code is written.
* **Expose Key Decisions:** Add the directive, *"For each step, show me the key decisions you'd make"* to the spec prompt, ensuring the AI locks into a single, verifiable path.
 
---
 
## **Phase 2: Execution Rules (Simplicity & Surgical Precision)**
 
* **Launch Parallel Sub-Agents:** Use the prompt *"Launch [X] sub-agents to handle this"* for independent tasks, high-volume research, or to get multiple perspectives on the same code block without the model anchoring to previous responses.
* **Enforce "Simplicity First":** Write only the minimum code required to solve the problem. Reject speculative features, unnecessary abstractions, and unused configurability. If 200 lines can be 50, rewrite it.
* **Make Surgical Changes:** Touch only what must be touched. Match existing styles perfectly, avoid refactoring unbroken code, and do not "improve" adjacent formatting.
* **Clean Up Only Your Mess:** Remove imports, variables, or functions orphaned by your specific changes, but explicitly leave pre-existing dead code alone unless asked.
 
---
 
## **Phase 3: Verification & Loops (Goal-Driven Execution)**
 
* **Transform Tasks into Verifiable Goals:** Define explicit success criteria before acting (e.g., "Write tests for invalid inputs, then make them pass" instead of "Add validation").
* **Map the Loop:** State a brief, step-by-step verification plan (e.g., `Step -> Verify -> Check`) to allow for independent looping without constant human clarification.
* **Trigger "Verify before you build":** Ensure your system instructions force the AI to state its verification plan before execution.
* **Enable Verification Tools:** Ask the AI which external tools (like deployment integrations) or internal tools (like brand voice validators) it needs to self-correct its own output.
* **Protect Human Validation Zones:** Identify high-risk areas (like payment processing) and completely remove the AI's autonomous execution permissions, requiring strict human sign-off.
 
---
 
## **Phase 4: Scaling & Automation**
 
* **Build Skills from Reality:** Use the prompt *"Based on this conversation, build me a skill"* to package manually validated, successfully completed workflows into repeatable instructions. Never build skills abstractly.
* **Log Your "Gotchas":** Instruct the AI to add a "Gotchas" section to every skill to record edge cases, stylistic quirks, and corrected mistakes so they are never repeated.
* **Evaluate "Automate this" Carefully:** Manage your operational debt by filtering every automation request through a strict taste and quality analysis.
 
**The Automation vs. Augmentation Filter:**
 
| Task Characteristic | Execution Decision |
| --- | --- |
| Requires human taste or nuance to judge output | **Augment** (Human-in-the-loop) |
| Output is purely quantifiable and objective | **Automate** (Fully autonomous) |
| AI output at 80% quality is acceptable | **Automate** (Fully autonomous) |
| AI output at 80% quality is completely unacceptable | **Augment** (Human-in-the-loop) |
