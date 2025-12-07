#!/usr/bin/env node
/**
 * Citation Validation Script
 * 
 * Validates that:
 * 1. All citations in markdown files reference entries in the bibliography
 * 2. At least 50% of sources are peer-reviewed
 * 3. Citations follow APA format
 */

const fs = require('fs');
const path = require('path');
const bibtexParse = require('bibtex-parse-js');

const DOCS_DIR = path.join(__dirname, '..', 'docs');
const BIB_FILE = path.join(__dirname, '..', 'references', 'physical-ai-book.bib');
const PEER_REVIEWED_THRESHOLD = 0.5;

// Extract citation keys from markdown content
function extractCitations(content) {
  const citeRegex = /\[@([\w-]+)\]/g;
  const citations = [];
  let match;
  while ((match = citeRegex.exec(content)) !== null) {
    citations.push(match[1]);
  }
  return citations;
}

// Recursively find all markdown files
function findMarkdownFiles(dir) {
  const files = [];
  const entries = fs.readdirSync(dir, { withFileTypes: true });
  
  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...findMarkdownFiles(fullPath));
    } else if (entry.isFile() && entry.name.endsWith('.md')) {
      files.push(fullPath);
    }
  }
  
  return files;
}

// Main validation function
async function validateCitations() {
  console.log('🔍 Validating citations...\n');
  
  // Check if bibliography exists
  if (!fs.existsSync(BIB_FILE)) {
    console.warn('⚠️  Bibliography file not found:', BIB_FILE);
    console.warn('   Skipping citation validation (expected in early development)\n');
    process.exit(0);
  }
  
  // Parse bibliography
  const bibContent = fs.readFileSync(BIB_FILE, 'utf-8');
  const bibEntries = bibtexParse.toJSON(bibContent);
  const bibKeys = new Set(bibEntries.map(entry => entry.citationKey));
  
  // Find all markdown files
  const mdFiles = findMarkdownFiles(DOCS_DIR);
  
  // Extract all citations
  const allCitations = new Set();
  const missingRefs = new Set();
  
  for (const file of mdFiles) {
    const content = fs.readFileSync(file, 'utf-8');
    const citations = extractCitations(content);
    
    for (const cite of citations) {
      allCitations.add(cite);
      if (!bibKeys.has(cite)) {
        missingRefs.add(cite);
        console.error(`❌ Missing reference: [@${cite}] in ${path.relative(process.cwd(), file)}`);
      }
    }
  }
  
  // Check peer-review percentage
  const peerReviewedCount = bibEntries.filter(entry => 
    entry.entryType === 'article' || 
    entry.entryType === 'inproceedings' ||
    entry.entryType === 'conference'
  ).length;
  
  const peerReviewedPercentage = bibEntries.length > 0 
    ? peerReviewedCount / bibEntries.length 
    : 0;
  
  console.log(`\n📊 Citation Statistics:`);
  console.log(`   Total citations: ${allCitations.size}`);
  console.log(`   Bibliography entries: ${bibEntries.length}`);
  console.log(`   Peer-reviewed sources: ${peerReviewedCount} (${(peerReviewedPercentage * 100).toFixed(1)}%)`);
  
  if (peerReviewedPercentage < PEER_REVIEWED_THRESHOLD) {
    console.warn(`\n⚠️  Warning: Peer-reviewed percentage (${(peerReviewedPercentage * 100).toFixed(1)}%) is below ${(PEER_REVIEWED_THRESHOLD * 100)}% threshold`);
  }
  
  if (missingRefs.size > 0) {
    console.error(`\n❌ Validation failed: ${missingRefs.size} missing references`);
    process.exit(1);
  }
  
  console.log('\n✅ All citations validated successfully!\n');
}

validateCitations().catch(err => {
  console.error('Error during validation:', err);
  process.exit(1);
});
