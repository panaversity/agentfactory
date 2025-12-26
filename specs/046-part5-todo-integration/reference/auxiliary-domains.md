# Auxiliary Domain Reference

**Purpose**: Canonical domain examples for pattern transfer across Part 5 lessons.
**Source**: Derived from spec.md and plan.md requirements
**Created**: 2025-12-26

---

## Why Auxiliary Domains?

The Todo running example teaches Python fundamentals through a task management app. However, students need to see the **same patterns** applied to multiple domains to understand transferability. Each auxiliary domain demonstrates that the patterns learned with Todo apply universally.

**Teaching principle**: "You're not learning Todo—you're learning Python patterns that work everywhere."

---

## Domain 1: Legal (Case Management)

### Entity: Case

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `case_number` | `str` | Yes | - | Unique case identifier (e.g., "2025-CV-1234") |
| `title` | `str` | Yes | - | Brief case description |
| `client_name` | `str` | Yes | - | Primary client name |
| `case_type` | `str` | No | `"civil"` | One of: "civil", "criminal", "corporate", "family" |
| `status` | `str` | No | `"open"` | One of: "open", "pending", "discovery", "trial", "closed" |
| `filing_date` | `datetime` | No | `None` | When case was filed |
| `deadline` | `datetime` | No | `None` | Next important deadline |
| `is_priority` | `bool` | No | `False` | High-priority flag |

### Python Examples

```python
# Dict representation (early chapters)
case = {
    "case_number": "2025-CV-1234",
    "title": "Smith v. Jones Contract Dispute",
    "client_name": "Smith Corp",
    "case_type": "civil",
    "status": "open",
    "is_priority": True
}

# Class representation (OOP chapters)
@dataclass
class Case:
    case_number: str
    title: str
    client_name: str
    case_type: str = "civil"
    status: str = "open"
    filing_date: datetime | None = None
    deadline: datetime | None = None
    is_priority: bool = False

    def close_case(self) -> None:
        """Close the case."""
        self.status = "closed"

    def is_overdue(self) -> bool:
        """Check if deadline has passed."""
        if self.deadline is None:
            return False
        return datetime.now() > self.deadline
```

### Common Operations

| Operation | Function Name | Example |
|-----------|--------------|---------|
| Add case | `add_case()` | `add_case(cases, case_number, title, client)` |
| Close case | `close_case()` | `close_case(case)` |
| List open | `list_open_cases()` | `list_open_cases(cases)` |
| Find by number | `find_case()` | `find_case(cases, "2025-CV-1234")` |

---

## Domain 2: Finance (Invoice Management)

### Entity: Invoice

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `invoice_id` | `str` | Yes | - | Unique invoice number (e.g., "INV-2025-001") |
| `client_name` | `str` | Yes | - | Client being billed |
| `amount` | `float` | Yes | - | Total amount due |
| `description` | `str` | No | `""` | Invoice description |
| `status` | `str` | No | `"pending"` | One of: "draft", "pending", "paid", "overdue" |
| `issue_date` | `datetime` | No | `None` | When invoice was issued |
| `due_date` | `datetime` | No | `None` | Payment deadline |
| `is_paid` | `bool` | No | `False` | Whether fully paid |

### Python Examples

```python
# Dict representation (early chapters)
invoice = {
    "invoice_id": "INV-2025-001",
    "client_name": "Acme Corp",
    "amount": 5000.00,
    "description": "Consulting services - Q1",
    "status": "pending",
    "is_paid": False
}

# Class representation (OOP chapters)
@dataclass
class Invoice:
    invoice_id: str
    client_name: str
    amount: float
    description: str = ""
    status: str = "pending"
    issue_date: datetime | None = None
    due_date: datetime | None = None
    is_paid: bool = False

    def mark_paid(self) -> None:
        """Mark invoice as paid."""
        self.is_paid = True
        self.status = "paid"

    def is_overdue(self) -> bool:
        """Check if payment is overdue."""
        if self.due_date is None or self.is_paid:
            return False
        return datetime.now() > self.due_date
```

### Common Operations

| Operation | Function Name | Example |
|-----------|--------------|---------|
| Add invoice | `add_invoice()` | `add_invoice(invoices, invoice_id, client, amount)` |
| Mark paid | `mark_paid()` | `mark_paid(invoice)` |
| List unpaid | `list_unpaid_invoices()` | `list_unpaid_invoices(invoices)` |
| Total outstanding | `total_outstanding()` | `total_outstanding(invoices)` |

---

## Domain 3: Healthcare (Appointment Management)

### Entity: Appointment

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `appointment_id` | `str` | Yes | - | Unique identifier (e.g., "APT-2025-0001") |
| `patient_name` | `str` | Yes | - | Patient name |
| `doctor_name` | `str` | Yes | - | Attending physician |
| `appointment_type` | `str` | No | `"checkup"` | One of: "checkup", "followup", "specialist", "emergency" |
| `status` | `str` | No | `"scheduled"` | One of: "scheduled", "confirmed", "completed", "cancelled" |
| `scheduled_time` | `datetime` | No | `None` | Appointment date/time |
| `notes` | `str` | No | `""` | Additional notes |
| `is_confirmed` | `bool` | No | `False` | Patient confirmation |

### Python Examples

```python
# Dict representation (early chapters)
appointment = {
    "appointment_id": "APT-2025-0001",
    "patient_name": "John Doe",
    "doctor_name": "Dr. Smith",
    "appointment_type": "checkup",
    "status": "scheduled",
    "is_confirmed": False
}

# Class representation (OOP chapters)
@dataclass
class Appointment:
    appointment_id: str
    patient_name: str
    doctor_name: str
    appointment_type: str = "checkup"
    status: str = "scheduled"
    scheduled_time: datetime | None = None
    notes: str = ""
    is_confirmed: bool = False

    def confirm(self) -> None:
        """Confirm the appointment."""
        self.is_confirmed = True
        self.status = "confirmed"

    def cancel(self) -> None:
        """Cancel the appointment."""
        self.status = "cancelled"
```

### Common Operations

| Operation | Function Name | Example |
|-----------|--------------|---------|
| Schedule | `schedule_appointment()` | `schedule_appointment(appointments, patient, doctor, time)` |
| Confirm | `confirm_appointment()` | `confirm_appointment(appointment)` |
| Cancel | `cancel_appointment()` | `cancel_appointment(appointment)` |
| List today | `list_todays_appointments()` | `list_todays_appointments(appointments)` |

---

## Domain 4: Marketing (Campaign Management)

### Entity: Campaign

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `campaign_id` | `str` | Yes | - | Unique identifier (e.g., "CAM-2025-Q1") |
| `name` | `str` | Yes | - | Campaign name |
| `channel` | `str` | No | `"email"` | One of: "email", "social", "ppc", "content" |
| `status` | `str` | No | `"draft"` | One of: "draft", "active", "paused", "completed" |
| `budget` | `float` | No | `0.0` | Allocated budget |
| `start_date` | `datetime` | No | `None` | Campaign start |
| `end_date` | `datetime` | No | `None` | Campaign end |
| `is_active` | `bool` | No | `False` | Currently running |

### Python Examples

```python
# Dict representation (early chapters)
campaign = {
    "campaign_id": "CAM-2025-Q1",
    "name": "Spring Product Launch",
    "channel": "email",
    "status": "draft",
    "budget": 10000.00,
    "is_active": False
}

# Class representation (OOP chapters)
@dataclass
class Campaign:
    campaign_id: str
    name: str
    channel: str = "email"
    status: str = "draft"
    budget: float = 0.0
    start_date: datetime | None = None
    end_date: datetime | None = None
    is_active: bool = False

    def launch(self) -> None:
        """Launch the campaign."""
        self.is_active = True
        self.status = "active"

    def pause(self) -> None:
        """Pause the campaign."""
        self.is_active = False
        self.status = "paused"
```

### Common Operations

| Operation | Function Name | Example |
|-----------|--------------|---------|
| Create | `create_campaign()` | `create_campaign(campaigns, name, channel, budget)` |
| Launch | `launch_campaign()` | `launch_campaign(campaign)` |
| Pause | `pause_campaign()` | `pause_campaign(campaign)` |
| List active | `list_active_campaigns()` | `list_active_campaigns(campaigns)` |

---

## Pattern Mapping: Todo → Domains

| Todo Pattern | Legal | Finance | Healthcare | Marketing |
|-------------|-------|---------|------------|-----------|
| `task` | `case` | `invoice` | `appointment` | `campaign` |
| `tasks` | `cases` | `invoices` | `appointments` | `campaigns` |
| `add_task()` | `add_case()` | `add_invoice()` | `schedule_appointment()` | `create_campaign()` |
| `complete_task()` | `close_case()` | `mark_paid()` | `complete_appointment()` | `complete_campaign()` |
| `is_done` | `is_closed` | `is_paid` | `is_confirmed` | `is_active` |
| `priority` | `is_priority` | `amount` | `appointment_type` | `budget` |
| `due_date` | `deadline` | `due_date` | `scheduled_time` | `end_date` |

---

## Usage Guidelines

1. **Early chapters (Ch16-22)**: Use dict representation for all domains
2. **Functions chapter (Ch23)**: Use functions like `add_case()`, `add_invoice()`
3. **OOP chapters (Ch27-28)**: Introduce class-based representations
4. **Dataclass chapter (Ch29)**: Show dataclass versions
5. **Later chapters (Ch30-32)**: Use validated/typed versions

### Domain Selection by Chapter Theme

| Chapter Theme | Primary Domain | Auxiliary Domains |
|--------------|----------------|-------------------|
| Data storage | Todo | Invoice (amounts), Case (text) |
| Date/time | Todo | Appointment (scheduling), Campaign (dates) |
| Validation | Todo | Invoice (amounts), Case (status) |
| Filtering | Todo | All domains (status filtering) |
| File I/O | Todo | Invoice (JSON export), Case (CSV reports) |

---

## Example: Pattern Transfer in Lesson

When teaching list comprehensions (Ch21-03):

```python
# Primary: Todo
pending_tasks = [task for task in tasks if not task["done"]]

# Auxiliary 1: Legal
open_cases = [case for case in cases if case["status"] == "open"]

# Auxiliary 2: Finance
unpaid_invoices = [inv for inv in invoices if not inv["is_paid"]]

# Auxiliary 3: Healthcare
todays_appointments = [apt for apt in appointments
                       if apt["scheduled_time"].date() == today]
```

**Teaching note**: "The pattern `[item for item in items if condition]` works the same way for tasks, cases, invoices, or any collection. You're learning a universal Python skill."
