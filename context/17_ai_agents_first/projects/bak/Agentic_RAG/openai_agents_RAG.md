# OpenAI Agents RAG Example

```python
!pip install -Uq openai-agents langchain-community chromadb "openai-agents[litellm]" langchain-openai
!pip install pypdf
```

```python
from google.colab import userdata
import os
os.environ["GEMINI_API_KEY"] = userdata.get('GEMINI_API_KEY')
```

```python
import nest_asyncio
nest_asyncio.apply()
```

```python
!pip install -Uq openai-agents langchain-community chromadb "openai-agents[litellm]" langchain-openai langchain-google-genai
```

```python
!pip install -Uq openai-agents chromadb google-genai
```

```python
import os
from google.colab import userdata

# Set your API keys (replace the placeholders with your actual keys)
# os.environ["OPENAI_API_KEY"] = "sk-..."            # OpenAI API key for Agents SDK
GEMINI_API_KEY = userdata.get("GEMINI_API_KEY")       # Google Gemini API key

# Import the necessary classes
from agents import Agent, Runner, function_tool   # OpenAI Agents SDK components
import chromadb                                   # ChromaDB client
from chromadb.utils import embedding_functions    # (optional, if using embedding functions directly)
from google import genai                          # Google GenAI SDK for Gemini
from google.genai.types import EmbedContentConfig
```

```python
# Initialize ChromaDB in-memory client
chroma_client = chromadb.Client()  # default uses an in-memory SQLite store

# Initialize Google GenAI client with the Gemini API key
client = genai.Client(api_key=GEMINI_API_KEY)
```

```python
# Define a few short text documents (e.g., Wikipedia-style snippets)
documents = [
    "Cats are small, domesticated carnivorous mammals often valued by humans for companionship and for their ability to hunt vermin.",
    "Dogs are domesticated mammals, not natural wild animals. They were originally bred from wolves.",
    "The Apollo program was a series of space missions by NASA in the 1960s and 1970s aimed at landing humans on the Moon."
]
doc_ids = ["doc1", "doc2", "doc3"]

# (Optional) Print the documents to verify
for i, doc in enumerate(documents, 1):
    print(f"Document {i}: {doc[:60]}...")
```

```python
# Embed each document using the Gemini embedding model
embed_model = "gemini-embedding-exp-03-07"

# Generate embeddings for all documents in one call
response = client.models.embed_content(
    model=embed_model,
    contents=documents,
    config=EmbedContentConfig(task_type="RETRIEVAL_DOCUMENT")  # optimize embeddings for retrieval
)

# Extract the embedding vectors from the response
doc_embeddings = [emb.values for emb in response.embeddings]

# Check the number of embeddings and dimensionality of one embedding
print(f"Generated {len(doc_embeddings)} embeddings.")
print(f"Dimension of first embedding: {len(doc_embeddings[0])}")
```

```python
print(f"Sample of first embedding vector: {doc_embeddings[0][:5]}...")  # print first 5 values
```

```python
# Create a ChromaDB collection for our documents
collection = chroma_client.create_collection(name="knowledge_base")

# Add documents, their embeddings, and IDs to the collection
collection.add(
    documents=documents,
    embeddings=doc_embeddings,
    ids=doc_ids
)

# (Optional) verify collection size
print("Documents in collection:", collection.count())
```

```python
# User's question
user_question = "What was the goal of the Apollo program?"

# Embed the user query using the same model (use task_type RETRIEVAL_QUERY for queries)
query_response = client.models.embed_content(
    model=embed_model,
    contents=[user_question],
    config=EmbedContentConfig(task_type="RETRIEVAL_QUERY")
)
query_vector = query_response.embeddings[0].values
# query_vector

# Use ChromaDB to find the most similar document(s) to the query
results = collection.query(
    query_embeddings=[query_vector],
    n_results=2,  # fetch top 2 most similar docs
    # Remove 'ids' from the include list as it's not a valid option
    include=["documents", "distances"]
)
results

# Print out the retrieved documents and their similarity scores
# print("Query:", user_question)
# for doc, score, doc_id in zip(results["documents"][0], results["distances"][0], results["ids"][0]):
#     print(f"- Retrieved {doc_id} with similarity score {score:.4f}: {doc[:60]}...")
```

```python
print(f"Sample of first embedding vector: {doc_embeddings[0][:5]}...")  # print first 5 values
```

```python
# Prepare the context from the retrieved docs
retrieved_docs = results["documents"][0]
context = "\n\n".join(retrieved_docs)

# Formulate the prompt for the LLM
prompt = f"""Use the following context to answer the question.

Context:
{context}

Question:
{user_question}

Answer the question using only the information from the context above."""
print(prompt)
```

```python
# Use the Gemini 1.5 Flash model to get an answer based on the context
response = client.models.generate_content(
    model="gemini-1.5-flash",
    contents=prompt
)
answer = response.text

print("Answer:", answer)
```

```python
import nest_asyncio
nest_asyncio.apply()
```

```python
import os
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
from agents.run import RunConfig
from google.colab import userdata


gemini_api_key = userdata.get("GEMINI_API_KEY")


# Check if the API key is present; if not, raise an error
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY is not set. Please ensure it is defined in your .env file.")

#Reference: https://ai.google.dev/gemini-api/docs/openai
external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_client
)

config = RunConfig(
    model=model,
    model_provider=external_client,
    tracing_disabled=True
)
```

```python
import os
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, set_default_openai_client, set_tracing_disabled
from agents.run import RunConfig
from google.colab import userdata

set_tracing_disabled(True)
gemini_api_key = userdata.get("GEMINI_API_KEY")


external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)
set_default_openai_client(external_client)
```

```python
from agents.tool import function_tool
```

```python
@function_tool
def answer_from_knowledge_base(query: str) -> str:
    """
    Tool: Given a user query, this tool searches the knowledge base and returns an answer using retrieved documents.
    """
    # Embed the query
    q_resp = client.models.embed_content(
        model=embed_model,
        contents=[query],
        config=EmbedContentConfig(task_type="RETRIEVAL_QUERY")
    )
    q_vector = q_resp.embeddings[0].values
    # Search the vector store
    res = collection.query(query_embeddings=[q_vector], n_results=1, include=["documents"])
    top_doc = res["documents"][0][0]  # top result's text
    # Construct prompt with retrieved context
    prompt = f"Context:\n{top_doc}\n\nQuestion:\n{query}\n\nAnswer the question using only the context above."
    # Generate answer with Gemini 1.5 Flash
    resp = client.models.generate_content(model="gemini-1.5-flash", contents=prompt)
    return resp.text
```

```python
!pip install -Uq openai-agents langchain-community chromadb langchain-openai langchain-google-genai litellm
```

```python
# Install necessary packages
!pip install -Uq openai-agents langchain-community chromadb langchain-openai langchain-google-genai litellm

# Import necessary libraries
import os
import asyncio
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, set_default_openai_client, set_tracing_disabled, function_tool
from google.colab import userdata
import chromadb
from google import genai
from google.genai.types import EmbedContentConfig
import nest_asyncio

# Apply nest_asyncio for compatibility with notebooks
nest_asyncio.apply()

# Set up tracing
set_tracing_disabled(True)

# Get Gemini API key
gemini_api_key = userdata.get("GEMINI_API_KEY")
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY is not set. Please ensure it is defined in your userdata.")

# Configure the OpenAI-compatible client for Gemini
external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Set this as the default OpenAI client for agents
set_default_openai_client(external_client)

# Initialize Google GenAI client for RAG embedding/generation
client = genai.Client(api_key=gemini_api_key)

# Initialize ChromaDB in-memory client
chroma_client = chromadb.Client()

# Define and embed documents
documents = [
    "Cats are small, domesticated carnivorous mammals often valued by humans for companionship and for their ability to hunt vermin.",
    "Dogs are domesticated mammals, not natural wild animals. They were originally bred from wolves.",
    "The Apollo program was a series of space missions by NASA in the 1960s and 1970s aimed at landing humans on the Moon."
]
doc_ids = ["doc1", "doc2", "doc3"]
embed_model = "gemini-embedding-exp-03-07" # Or whichever model you used

# Generate embeddings for all documents in one call
response = client.models.embed_content(
    model=embed_model,
    contents=documents,
    config=EmbedContentConfig(task_type="RETRIEVAL_DOCUMENT")
)
doc_embeddings = [emb.values for emb in response.embeddings]

# Create or get the collection
collection = chroma_client.get_or_create_collection(name="knowledge_base1")

# Add documents, handling potential duplicates
try:
    collection.add(
        documents=documents,
        embeddings=doc_embeddings,
        ids=doc_ids
    )
except Exception as e:
    print(f"Could not add documents to collection, potentially they already exist: {e}")


# Define the RAG tool using the accessible clients and variables
@function_tool
def answer_from_knowledge_base(query: str) -> str:
    """
    Tool: Given a user query, this tool searches the knowledge base and returns an answer using retrieved documents.
    """
    print(f"[Debug] RAG function call with query {query}")
    # Embed the query using the accessible 'client' and 'embed_model'
    q_resp = client.models.embed_content(
        model=embed_model,
        contents=[query],
        config=EmbedContentConfig(task_type="RETRIEVAL_QUERY")
    )
    q_vector = q_resp.embeddings[0].values
    # Search the vector store using the accessible 'collection'
    res = collection.query(query_embeddings=[q_vector], n_results=1, include=["documents"])
    print(f"[Debug] RAG vector db output {res}")
    # Check if any documents were returned
    if res and res.get("documents") and res["documents"][0]:
        top_doc = res["documents"][0][0]  # top result's text
        # Construct prompt with retrieved context
        prompt = f"Context:\n{top_doc}\n\nQuestion:\n{query}\n\nAnswer the question using only the context above."
        # Generate answer with Gemini 1.5 Flash using the accessible 'client'
        resp = client.models.generate_content(model="gemini-1.5-flash", contents=prompt)
        print(f"[Debug] RAG function call with response ***{resp.text}***")
        return resp.text
    else:
        return "Could not find relevant information in the knowledge base."

# Create an agent that can use the knowledge base tool
# Use OpenAIChatCompletionsModel with the external_client
qa_agent = Agent(
    name="QA Agent",
    instructions="You are a helpful assistant. If the user asks a question, use your tools to find information in the knowledge base and answer with that information.",
    tools=[answer_from_knowledge_base],
    # Use OpenAIChatCompletionsModel with the pre-configured external_client
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash-001", # Specify the model name compatible with the OpenAI-like endpoint
        openai_client=external_client
    )
)

async def main():
    agent_question = "Which domestic animal was originally bred from wolves? what do you know about Apollo?"

    # Run the agent
    result = await Runner.run(qa_agent, agent_question)

    # Extract and print the final answer
    print("Agent result:", result)
    print("Agent's answer:", result.final_output)

if __name__ == "__main__":
    asyncio.run(main())
```

## With PDF

```python
!pip install pypdf
from langchain_community.document_loaders import PyPDFLoader

def load_and_split_pdf(file_path: str):
  """Loads a PDF and splits it into pages."""
  loader = PyPDFLoader(file_path)
  pages = loader.load_and_split()
  return pages

# Example usage: Upload a PDF and process it
from google.colab import files
uploaded = files.upload()

pdf_file_path = list(uploaded.keys())[0]
pdf_pages = load_and_split_pdf(pdf_file_path)

print(f"Loaded {len(pdf_pages)} pages from {pdf_file_path}")

# Extract text content from the pages
pdf_documents_text = [page.page_content for page in pdf_pages]
pdf_doc_ids = [f"pdf_page_{i+1}" for i in range(len(pdf_pages))]

# Embed the PDF document content
pdf_embeddings_response = client.models.embed_content(
    model=embed_model,
    contents=pdf_documents_text,
    config=EmbedContentConfig(task_type="RETRIEVAL_DOCUMENT")
)
pdf_doc_embeddings = [emb.values for emb in pdf_embeddings_response.embeddings]

# Add the PDF content and embeddings to the ChromaDB collection
# Get the existing collection or create a new one if it doesn't exist
collection = chroma_client.get_or_create_collection(name="knowledge_base1")

try:
    collection.add(
        documents=pdf_documents_text,
        embeddings=pdf_doc_embeddings,
        ids=pdf_doc_ids
    )
    print(f"Added {len(pdf_pages)} PDF pages to the knowledge base.")
except Exception as e:
    print(f"Could not add PDF documents to collection, potentially they already exist: {e}")

print("Total documents in collection:", collection.count())

# The rest of your agent and RAG tool code should work with the updated collection.
# You can now ask questions that might be answered by the content of the uploaded PDF.
```

```python
async def main_pdf_question():
    # Ask a question that relates to the uploaded PDF content
    pdf_question = "Who is Muhammad Qasim" # Replace with a specific question about your PDF

    # Run the agent with the new question
    result = await Runner.run(qa_agent, pdf_question)

    # Extract and print the final answer
    print("Agent result for PDF question:", result)
    print("Agent's answer for PDF question:", result.final_output)

if __name__ == "__main__":
    asyncio.run(main_pdf_question())
```
