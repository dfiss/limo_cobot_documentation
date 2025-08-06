---
sidebar_position: 1
---

# Introduction to LIMO Cobot

Welcome to the **LIMO Cobot Documentation** - your comprehensive guide to building advanced applications with our collaborative robotics platform.

## What is LIMO Cobot?

LIMO Cobot is a cutting-edge collaborative robotics platform designed for seamless integration into various industrial and research environments. Our platform combines advanced motion control, intelligent perception, and intuitive programming interfaces to deliver exceptional performance and safety.

### Key Features

- **🤖 Advanced Motion Control** - Precision robotics with safety-first design
- **🔧 Easy Integration** - RESTful APIs and SDKs for multiple programming languages  
- **📊 Real-time Monitoring** - Comprehensive telemetry and diagnostics
- **🛡️ Safety First** - Built-in collision detection and emergency stop systems
- **🔄 Flexible Configuration** - Customizable workflows and task automation

## Getting Started

Choose your path to get started with LIMO Cobot:

### Quick Start Guide

New to LIMO Cobot? Start here for a rapid introduction:

```bash
# Install the LIMO Cobot SDK
npm install @limo/cobot-sdk

# Or using Python
pip install limo-cobot
```

### For Developers

- **[Installation Guide](./tutorial-basics/create-a-document.md)** - Set up your development environment
- **[API Reference](./tutorial-basics/create-a-page.md)** - Complete API documentation
- **[Code Examples](./tutorial-basics/markdown-features.mdx)** - Ready-to-use code snippets

### For System Integrators

- **[Configuration Guide](./tutorial-basics/deploy-your-site.md)** - System setup and configuration
- **[Safety Guidelines](./tutorial-basics/congratulations.md)** - Important safety considerations
- **[Troubleshooting](./tutorial-extras/manage-docs-versions.md)** - Common issues and solutions

## System Requirements

Before getting started, ensure your system meets these requirements:

- **Operating System**: Windows 10/11, Ubuntu 20.04+, or macOS 11+
- **Memory**: Minimum 8GB RAM (16GB recommended)
- **Network**: Ethernet connection for robot communication
- **Development Environment**: Node.js 18+ or Python 3.8+

## Architecture Overview

LIMO Cobot follows a modular architecture designed for scalability and reliability:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Application   │    │   LIMO Cobot    │    │   Robot Arm     │
│   Layer         │◄──►│   Platform      │◄──►│   Hardware      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Next Steps

Ready to dive in? Here's what to do next:

1. **[Install the SDK](./tutorial-basics/create-a-document.md)** - Get the development tools
2. **[Run Your First Program](./tutorial-basics/create-a-page.md)** - Hello World example
3. **[Explore the API](./tutorial-basics/markdown-features.mdx)** - Learn the core concepts

:::tip Need Help?

Join our [community forum](https://github.com/your-org/limo-cobot-documentation/discussions) or check out our [Discord server](https://discord.gg/your-discord) for support and discussions.

:::

The command also installs all necessary dependencies you need to run Docusaurus.

## Start your site

Run the development server:

```bash
cd my-website
npm run start
```

The `cd` command changes the directory you're working with. In order to work with your newly created Docusaurus site, you'll need to navigate the terminal there.

The `npm run start` command builds your website locally and serves it through a development server, ready for you to view at http://localhost:3000/.

Open `docs/intro.md` (this page) and edit some lines: the site **reloads automatically** and displays your changes.
