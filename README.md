# Physical AI & Humanoid Robotics

[![Build Status](https://github.com/your-organization/physical-ai-book/workflows/Build%20and%20Test/badge.svg)](https://github.com/your-organization/physical-ai-book/actions)
[![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](LICENSE.content.md)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE.code.md)

**A Graduate-Level Technical Textbook**

This open-source textbook provides a comprehensive introduction to Physical AI and Humanoid Robotics, covering foundational concepts, simulation tools, and modern vision-language-action (VLA) models for robot control.

## Book Structure

The book is organized into six modules:

- **Module 0: Physical AI Foundations** - Embodied AI, subsumption architecture, and probabilistic robotics
- **Module 1: ROS 2 (Robotic Nervous System)** - Robot Operating System 2 fundamentals
- **Module 2: Gazebo & Unity (Digital Twin)** - Robot simulation and digital twin development
- **Module 3: NVIDIA Isaac (AI-Robot Brain)** - Isaac Sim, Isaac Gym, and Isaac ROS
- **Module 4: Vision-Language-Action (VLA)** - Foundation models for robotics (OpenVLA, SmolVLA)
- **Module 5: Capstone Project** - End-to-end humanoid robot project

## Getting Started

### Prerequisites

- **Node.js**: 18.x or higher
- **npm**: 9.x or higher
- **Python**: 3.10 or higher (for code examples)
- **Docker**: 20.x or higher (optional, for code examples)

### Local Development

1. Clone the repository:
   ```bash
   git clone https://github.com/your-organization/physical-ai-book.git
   cd physical-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

   This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

4. Build the static site:
   ```bash
   npm run build
   ```

   This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Contributing

We welcome contributions from the robotics and AI community! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on:

- Reporting issues
- Suggesting new content or improvements
- Submitting pull requests
- Code of conduct

## License

This project uses a dual-license structure:

- **Book Content** (all files in `docs/`): Licensed under [Creative Commons Attribution-ShareAlike 4.0 International License (CC BY-SA 4.0)](LICENSE.content.md)
- **Code Examples** (all files in `examples/`): Licensed under [Apache License 2.0](LICENSE.code.md)

## Citation

If you use this textbook in your research or teaching, please cite it as:

```bibtex
@book{PhysicalAIBook2025,
  title = {Physical AI \& Humanoid Robotics: A Graduate-Level Technical Textbook},
  author = {{Physical AI \& Humanoid Robotics Book Contributors}},
  year = {2025},
  publisher = {GitHub},
  url = {https://github.com/your-organization/physical-ai-book},
  note = {Content licensed under CC BY-SA 4.0, Code licensed under Apache 2.0}
}
```

## Acknowledgments

This textbook builds upon decades of robotics research and incorporates content from:

- Brooks, R. A. (1991). Intelligence without representation
- Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics
- Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics
- And many other contributors to the field of robotics and AI

Full bibliography available in [docs/references/bibliography.md](docs/references/bibliography.md).

## Contact

For questions or feedback:

- Open an issue on [GitHub Issues](https://github.com/your-organization/physical-ai-book/issues)
- Join discussions on [GitHub Discussions](https://github.com/your-organization/physical-ai-book/discussions)

---

Built with [Docusaurus](https://docusaurus.io/).
